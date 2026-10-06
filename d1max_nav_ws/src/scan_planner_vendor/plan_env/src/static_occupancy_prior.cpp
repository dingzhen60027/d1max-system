#include <plan_env/static_occupancy_prior.hpp>
#include <openssl/evp.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace scan_planner {
namespace {
constexpr std::size_t kMaximumBytes=256ULL*1024*1024;
bool digestValid(const std::string &value) {
  if(value.size()!=64)return false;
  for(char c:value)if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')))return false;
  return true;
}
void require(bool valid,const char *message) {
  if(!valid)throw std::invalid_argument(std::string("static occupancy prior: ")+message);
}
std::vector<std::uint8_t> readFile(const std::filesystem::path &path,std::size_t bound) {
  std::ifstream file(path,std::ios::binary|std::ios::ate);
  require(bool(file),"file unavailable");
  const auto size=file.tellg();
  require(size>0&&static_cast<std::uint64_t>(size)<=bound,"invalid file length");
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  file.seekg(0);file.read(reinterpret_cast<char*>(bytes.data()),bytes.size());
  require(bool(file),"short file read");return bytes;
}
bool identityTransform(const nlohmann::json &translation,const nlohmann::json &rotation) {
  if(!translation.is_array()||translation.size()!=3||!rotation.is_array()||rotation.size()!=4)return false;
  for(std::size_t i=0;i<3;++i)
    if(!translation[i].is_number()||!std::isfinite(translation[i].get<double>())||translation[i].get<double>()!=0.)return false;
  for(std::size_t i=0;i<4;++i)
    if(!rotation[i].is_number()||!std::isfinite(rotation[i].get<double>())||rotation[i].get<double>()!=(i==3?1.:0.))return false;
  return true;
}
std::array<double,3> finiteTriple(const nlohmann::json &value) {
  require(value.is_array()&&value.size()==3,"invalid bounds");
  std::array<double,3> out{};
  for(std::size_t i=0;i<3;++i) {
    require(value[i].is_number(),"nonnumeric bounds");out[i]=value[i].get<double>();
    require(std::isfinite(out[i]),"nonfinite bounds");
  }
  return out;
}
} // namespace

std::string StaticOccupancyPrior::sha256(const void *data,std::size_t size) {
  std::array<unsigned char,EVP_MAX_MD_SIZE> result{};unsigned count=0;
  if(EVP_Digest(data,size,result.data(),&count,EVP_sha256(),nullptr)!=1||count!=32)
    throw std::runtime_error("SHA256 unavailable");
  std::ostringstream out;out<<std::hex<<std::setfill('0');
  for(unsigned i=0;i<count;++i)out<<std::setw(2)<<unsigned(result[i]);return out.str();
}

std::shared_ptr<const StaticOccupancyPrior> StaticOccupancyPrior::load(
    const std::string &manifest_path,const Expected &expected) {
  require(digestValid(expected.manifest_sha256)&&digestValid(expected.geometry_sha256),"expected SHA256 required");
  require(!expected.frame_id.empty()&&!expected.odom_frame_id.empty()&&!expected.map_version.empty()&&expected.map_version.size()<=256,
      "expected frame and map identity required");
  require(expected.transform_contract=="fixed_identity_map_from_odom_v1", "unsupported transform contract");
  require(std::isfinite(expected.resolution)&&expected.resolution>0.,"invalid expected resolution");
  require(expected.allow_floor_contact||expected.support_endpoint_error_bound_m==1e-5,
      "nondefault floor endpoint bound requires opt-in floor contact");
  const auto bytes=readFile(manifest_path,1024*1024);
  require(sha256(bytes.data(),bytes.size())==expected.manifest_sha256,"manifest SHA256 mismatch");
  const auto manifest=nlohmann::json::parse(bytes.begin(),bytes.end());
  require(manifest.at("schema")==1&&manifest.at("kind")=="certified_static_occupancy_prior","unsupported schema");
  const auto provenance=manifest.at("provenance").get<std::string>();
  require(provenance=="isaac_closed_collision_geometry_v1"||provenance=="certified_prebuilt_static_volume_v1",
      "uncertified free-space provenance");
  require(manifest.at("frame_id")==expected.frame_id&&manifest.at("map_version")==expected.map_version,
      "map identity mismatch");
  auto codes=nlohmann::json{{"free",0},{"occupied",1},{"unknown",2}};
  if(expected.allow_floor_contact)codes["support_contact"]=3;
  require(manifest.at("storage_order")=="C_xyz_z_fastest"&&manifest.at("state_codes")==codes,"invalid encoding");
  require(manifest.at("dtype")=="uint8"&&manifest.at("meters_per_unit")==1.&&manifest.at("up_axis")=="Z",
      "invalid metric voxel frame");
  require(manifest.at("voxel_resolution").is_number()&&
      manifest.at("voxel_resolution").get<double>()==expected.resolution,"resolution mismatch");
  for(const auto *field:{"scene_sha256","spec_sha256","collider_sha256","data_sha256"})
    require(digestValid(manifest.at(field).get<std::string>()),"missing source hash");
  require(manifest.at("collider_sha256")==expected.geometry_sha256,"geometry SHA256 mismatch");
  const auto &transform=manifest.at("map_from_odom");
  require(transform.at("transform_contract")==expected.transform_contract&&
      transform.at("from_frame")==expected.odom_frame_id&&transform.at("to_frame")==expected.frame_id&&
      identityTransform(transform.at("translation"),transform.at("rotation_xyzw")),"nonidentity map transform");
  const auto &bounds=manifest.at("closed_world_bounds");
  require(bounds.at("semantics")=="whole_closed_cell_strictly_inside","uncertified closed volume semantics");
  const auto lower=finiteTriple(bounds.at("min")),upper=finiteTriple(bounds.at("max"));
  const double margin=manifest.at("geometry_margin_m").get<double>();
  require(std::isfinite(margin)&&margin>=0.&&margin<=expected.resolution,"invalid uncertainty margin");
  std::vector<std::array<double,6>> support_exclusions;
  if(expected.allow_floor_contact) {
    require(provenance=="isaac_closed_collision_geometry_v1"&&digestValid(expected.dynamic_registry_sha256),
        "floor contact requires simulation dynamic registry");
    require(std::isfinite(expected.support_floor_z)&&std::isfinite(expected.support_penetration_m)&&
        expected.support_penetration_m>0.&&expected.support_penetration_m<=.02,"invalid support floor parameters");
    require(std::isfinite(expected.support_endpoint_error_bound_m)&&expected.support_endpoint_error_bound_m>=1e-5&&
        expected.support_endpoint_error_bound_m<=std::min(.001,expected.resolution/20.),
        "invalid bounded support endpoint numerical contract");
    const auto &contact=manifest.at("flat_support_contact"),&geometry=manifest.at("collision_geometry");
    double endpoint_error=1e-5;
    if(contact.contains("floor_endpoint_error_bound_m")) {
      require(contact.at("floor_endpoint_error_bound_m").is_number(),"nonnumeric floor endpoint error bound");
      endpoint_error=contact.at("floor_endpoint_error_bound_m").get<double>();
    }
    require(endpoint_error==expected.support_endpoint_error_bound_m,
        "floor endpoint numerical contract does not match expected certificate");
    const auto &floor=geometry.at("floor");
    require(contact.at("schema")==1&&contact.at("kind")=="flat_plane_support_contact_v1"&&
        contact.at("floor_path")=="/World/GroundPlane/collisionPlane"&&
        contact.at("floor_z")==expected.support_floor_z&&contact.at("penetration_allowance_m")==expected.support_penetration_m&&
        contact.at("semantics")=="whole_closed_cell_floor_intersection_inside_authorized_xy_and_disjoint_from_nonfloor_static_solids_plus_margin",
        "uncertified floor contact semantics");
    require(geometry.at("flat_support_contact")==contact&&
        geometry.at("closed_world_bounds")==bounds&&expected.support_floor_z==lower[2]&&
        geometry.at("dynamic_actor_registry_sha256")==expected.dynamic_registry_sha256&&
        manifest.at("dynamic_actor_registry_sha256")==expected.dynamic_registry_sha256&&
        geometry.at("dynamic_actor_registry").is_array()&&geometry.at("dynamic_actor_registry").size()<=64,
        "floor contact dynamic registry mismatch");
    require(floor.at("path")==contact.at("floor_path")&&floor.at("type")=="Plane"&&
        floor.at("axis")=="Z"&&floor.at("extent")=="infinite"&&floor.at("z")==expected.support_floor_z,
        "floor contact must be the exact infinite flat plane");
    const auto &boxes=geometry.at("boxes");
    require(boxes.is_array()&&boxes.size()<=4096,"invalid floor exclusion geometry");
    for(const auto &box:boxes) {
      require(box.at("type")=="Cube","unsupported floor exclusion shape");
      const auto lo=finiteTriple(box.at("min")),hi=finiteTriple(box.at("max"));
      std::array<double,6> exclusion{};
      for(std::size_t d=0;d<3;++d) {
        require(lo[d]<hi[d],"empty floor exclusion");
        exclusion[d]=lo[d]-margin-1e-8;exclusion[d+3]=hi[d]+margin+1e-8;
      }
      support_exclusions.push_back(exclusion);
    }
  } else require(!manifest.contains("flat_support_contact")||manifest.at("flat_support_contact").is_null(),
      "floor contact is not enabled");
  const auto &origin=manifest.at("origin_index"),&shape=manifest.at("shape");
  require(origin.is_array()&&origin.size()==3&&shape.is_array()&&shape.size()==3,"invalid dimensions");
  auto out=std::shared_ptr<StaticOccupancyPrior>(new StaticOccupancyPrior);out->expected_=expected;
  if(manifest.contains("collision_geometry")&&manifest.at("collision_geometry").contains("dynamic_actor_registry")) {
    const auto &geometry=manifest.at("collision_geometry"),&actors=geometry.at("dynamic_actor_registry");
    out->dynamic_registry_sha256_=geometry.at("dynamic_actor_registry_sha256").get<std::string>();
    require(digestValid(out->dynamic_registry_sha256_)&&manifest.at("dynamic_actor_registry_sha256")==out->dynamic_registry_sha256_&&
        actors.is_array()&&actors.size()<=64,"invalid dynamic actor registry identity");
    for(const auto &actor:actors) {
      const auto id=actor.at("id").get<std::string>();
      require(!id.empty()&&id.size()<=128&&out->dynamic_actor_ids_.insert(id).second,"duplicate/invalid dynamic actor ID");
    }
  }
  if(!expected.dynamic_registry_sha256.empty())require(out->dynamic_registry_sha256_==expected.dynamic_registry_sha256,
      "expected dynamic registry identity mismatch");
  std::size_t total=1;
  for(std::size_t d=0;d<3;++d) {
    require(origin[d].is_number_integer()&&shape[d].is_number_integer(),"noninteger dimensions");
    require(!origin[d].is_number_unsigned()||origin[d].get<std::uint64_t>()<=static_cast<std::uint64_t>(std::numeric_limits<int>::max()),
        "unsigned world index exceeds supported range");
    const auto offset=origin[d].get<std::int64_t>(),count=shape[d].get<std::int64_t>();
    require(count>0&&count<=100000&&offset>=std::numeric_limits<int>::min()&&
        offset<=std::numeric_limits<int>::max()-count,"dimension range exceeded");
    require(total<=kMaximumBytes/static_cast<std::size_t>(count),"volume exceeds memory bound");
    require(lower[d]<upper[d]&&upper[d]-lower[d]>2*margin,"empty authorized volume");
    out->origin_[d]=static_cast<int>(offset);out->shape_[d]=static_cast<std::size_t>(count);total*=count;
  }
  const auto file=manifest.at("data_file").get<std::string>();
  const std::filesystem::path relative(file);
  require(!file.empty()&&!relative.is_absolute()&&relative.filename()==relative&&file!="."&&file!="..",
      "data file must be a manifest-relative basename");
  out->data_=readFile(std::filesystem::path(manifest_path).parent_path()/relative,kMaximumBytes);
  require(out->data_.size()==total,"dimension and byte count mismatch");
  require(manifest.at("data_size_bytes").is_number_unsigned()&&
      manifest.at("data_size_bytes").get<std::uint64_t>()==total,"declared byte count mismatch");
  require(sha256(out->data_.data(),out->data_.size())==manifest.at("data_sha256"),"data SHA256 mismatch");
  // A producer cannot claim FREE at/outside the authorization boundary even
  // with a valid hash. OCC and UNKNOWN may extend beyond the closed volume.
  for(std::size_t x=0;x<out->shape_[0];++x)for(std::size_t y=0;y<out->shape_[1];++y)
    for(std::size_t z=0;z<out->shape_[2];++z) {
      const auto state=out->data_[(x*out->shape_[1]+y)*out->shape_[2]+z];
      require(state<=2||(expected.allow_floor_contact&&state==3),"unknown state code");
      if(state!=0&&state!=3)continue;
      const std::array<std::size_t,3> index{{x,y,z}};
      std::array<double,3> cell_lo{},cell_hi{};
      for(std::size_t d=0;d<3;++d) {
        const double lo=(out->origin_[d]+static_cast<double>(index[d]))*expected.resolution;
        const double hi=lo+expected.resolution;
        cell_lo[d]=lo;cell_hi[d]=hi;
        if(state==0||d<2)require(lo>lower[d]+margin&&hi<upper[d]-margin,
            "authorized cell crosses closed-volume uncertainty boundary");
      }
      if(state==3) {
        require(cell_lo[2]<=expected.support_floor_z+1e-8&&cell_hi[2]>=expected.support_floor_z-1e-8,
            "support cell does not intersect the exact floor plane");
        for(const auto &box:support_exclusions) {
          bool disjoint=false;
          for(std::size_t d=0;d<3;++d)disjoint|=cell_hi[d]<box[d]||cell_lo[d]>box[d+3];
          require(disjoint,"support cell intersects nonfloor geometry uncertainty");
        }
      }
    }
  return out;
}

int StaticOccupancyPrior::state(const std::array<int,3> &world_index) const {
  std::array<std::size_t,3> cell{};
  for(std::size_t d=0;d<3;++d) {
    const auto offset=static_cast<std::int64_t>(world_index[d])-origin_[d];
    if(offset<0||static_cast<std::uint64_t>(offset)>=shape_[d])return 2;
    cell[d]=static_cast<std::size_t>(offset);
  }
  return data_[(cell[0]*shape_[1]+cell[1])*shape_[2]+cell[2]];
}

const std::uint8_t* StaticOccupancyPrior::columnStates(const std::array<int,3>& first,int high_z) const {
  if(high_z<first[2])return nullptr;
  std::array<std::size_t,3> cell{};
  for(std::size_t d=0;d<3;++d) {
    const auto offset=static_cast<std::int64_t>(first[d])-origin_[d];
    if(offset<0||static_cast<std::uint64_t>(offset)>=shape_[d])return nullptr;
    cell[d]=static_cast<std::size_t>(offset);
  }
  const auto end=static_cast<std::int64_t>(high_z)-origin_[2];
  if(end<0||static_cast<std::uint64_t>(end)>=shape_[2])return nullptr;
  return data_.data()+(cell[0]*shape_[1]+cell[1])*shape_[2]+cell[2];
}

bool StaticOccupancyPrior::matchesContext(const nlohmann::json &context) const {
  try {
    return context.at("map_version")==expected_.map_version&&
        context.at("static_prior_manifest_sha256")==expected_.manifest_sha256&&
        context.at("map_from_odom_contract")==expected_.transform_contract&&
        identityTransform(context.at("map_from_odom_translation"),context.at("map_from_odom_rotation_xyzw"));
  } catch(const nlohmann::json::exception &) {return false;}
}
} // namespace scan_planner
