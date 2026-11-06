// Exact frozen-mesh queries. No ROS, source clocks, model policy or live state.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <time.h>
#include <vector>

namespace py = pybind11;
template<class T> using Array = py::array_t<T, py::array::c_style>;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Python time.monotonic uses CLOCK_MONOTONIC on this Linux-only ROS target.
// Do not start a fresh duration budget after crossing the Python boundary.
static bool expired(double deadline) noexcept {
  timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) return true;
  const double now = static_cast<double>(value.tv_sec) +
                     static_cast<double>(value.tv_nsec) * 1e-9;
  return !(now <= deadline);  // NaN deadline also fails closed.
}

static void shape(const py::buffer_info& info, std::initializer_list<py::ssize_t> expected) {
  if (info.ndim != static_cast<py::ssize_t>(expected.size()))
    throw std::invalid_argument("invalid frozen mesh array rank");
  size_t axis = 0;
  for (const auto size : expected) {
    if (size >= 0 && info.shape[axis] != size)
      throw std::invalid_argument("invalid frozen mesh array shape");
    ++axis;
  }
}
template<class T> static void immutable(const Array<T>& array) {
  if (array.writeable()) throw std::invalid_argument("mesh inputs must be read-only");
}

class Mesh {
 public:
  Mesh(Array<float> triangles, Array<double> edges, Array<double> scale,
       Array<double> lower, Array<double> upper, Array<int32_t> children,
       Array<int32_t> spans, Array<py::ssize_t> indices)
      : triangles_(std::move(triangles)), edges_(std::move(edges)), scale_(std::move(scale)),
        lower_(std::move(lower)), upper_(std::move(upper)), children_(std::move(children)),
        spans_(std::move(spans)), indices_(std::move(indices)) {
    immutable(triangles_); immutable(edges_); immutable(scale_); immutable(lower_);
    immutable(upper_); immutable(children_); immutable(spans_); immutable(indices_);
    const auto tri = triangles_.request(), edge = edges_.request(), sc = scale_.request();
    const auto lo = lower_.request(), hi = upper_.request(), ch = children_.request();
    const auto sp = spans_.request(), ix = indices_.request();
    shape(tri, {-1,3,3}); triangle_count_ = tri.shape[0];
    shape(edge, {triangle_count_,2,3}); shape(sc,{3});
    shape(lo,{-1,3}); node_count_ = lo.shape[0]; shape(hi,{node_count_,3});
    shape(ch,{node_count_,2}); shape(sp,{node_count_,2}); shape(ix,{triangle_count_});
    if (!node_count_ || !triangle_count_) throw std::invalid_argument("empty mesh hierarchy");
    tri_ = static_cast<const float*>(tri.ptr); edge_ = static_cast<const double*>(edge.ptr);
    scale_data_ = static_cast<const double*>(sc.ptr); lo_ = static_cast<const double*>(lo.ptr);
    hi_ = static_cast<const double*>(hi.ptr); child_ = static_cast<const int32_t*>(ch.ptr);
    span_ = static_cast<const int32_t*>(sp.ptr); index_ = static_cast<const py::ssize_t*>(ix.ptr);
    // Validate once at warmup; a malformed/cyclic index must never be followed
    // with the GIL released. Immutable owners remain alive for this handle.
    for (py::ssize_t node = 0; node < node_count_; ++node) {
      const auto left=child_[2*node], right=child_[2*node+1];
      if ((left < 0) != (right < 0) ||
          (left >= 0 && (left <= node || right <= node || left >= node_count_ || right >= node_count_)))
        throw std::invalid_argument("invalid mesh child index");
      if (span_[2*node] < 0 || span_[2*node+1] < span_[2*node] || span_[2*node+1] > triangle_count_)
        throw std::invalid_argument("invalid mesh leaf span");
      for (int axis=0; axis<3; ++axis)
        if (!std::isfinite(lo_[3*node+axis]) || !std::isfinite(hi_[3*node+axis]) ||
            lo_[3*node+axis] > hi_[3*node+axis])
          throw std::invalid_argument("invalid mesh bounds");
    }
    for (py::ssize_t slot=0; slot<triangle_count_; ++slot)
      if (index_[slot] < 0 || index_[slot] >= triangle_count_)
        throw std::invalid_argument("invalid mesh triangle index");
    for (int axis=0; axis<3; ++axis)
      if (!std::isfinite(scale_data_[axis])) throw std::invalid_argument("invalid mesh scale");
  }

  py::tuple surface_candidates(Array<double> points, double padding, double deadline) const {
    immutable(points); const auto info=points.request(); shape(info,{-1,3});
    const auto count=info.shape[0]; const auto* xyz=static_cast<const double*>(info.ptr);
    py::array_t<bool> result(count); auto* output=result.mutable_data();
    std::fill(output,output+count,false); bool timed_out=false;
    {
      py::gil_scoped_release release;
      std::vector<int32_t> pending; pending.reserve(64); size_t work=0;
      timed_out=expired(deadline);
      for (py::ssize_t point=0; point<count && !timed_out; ++point) {
        const double* p=xyz+3*point;
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) continue;
        pending.clear(); pending.push_back(0);
        while (!pending.empty()) {
          if ((work++ & 127U)==0 && expired(deadline)) {timed_out=true; break;}
          const auto node=pending.back(); pending.pop_back(); bool inside=true;
          for (int axis=0; axis<3; ++axis)
            inside &= p[axis]>=lo_[3*node+axis]-padding-1e-9 &&
                      p[axis]<=hi_[3*node+axis]+padding+1e-9;
          if (!inside) continue;
          const auto left=child_[2*node];
          if (left<0) {output[point]=true; break;}
          pending.push_back(child_[2*node+1]); pending.push_back(left);
        }
      }
      timed_out = timed_out || expired(deadline);
      if (timed_out) std::fill(output,output+count,false);
    }
    return py::make_tuple(std::move(result),timed_out);
  }

  py::tuple first_intersections(Array<double> origins, Array<double> endpoints,
                               double padding, double deadline, bool checked=false) const {
    immutable(origins); immutable(endpoints);
    const auto oi=origins.request(), ei=endpoints.request(); shape(ei,{-1,3});
    const auto count=ei.shape[0]; shape(oi,{count,3});
    const auto* starts=static_cast<const double*>(oi.ptr);
    const auto* ends=static_cast<const double*>(ei.ptr);
    py::array_t<double> result(count); auto* output=result.mutable_data();
    std::fill(output,output+count,kInf); bool timed_out=false, needs_oracle=false;
    {
      py::gil_scoped_release release;
      // Query-local storage only. No conclusion, direction or first-hit cache
      // is shared with another ray, source, acquisition or worker.
      std::vector<int32_t> layer,next; layer.reserve(64); next.reserve(64);
      size_t work=0; timed_out=expired(deadline);
      for (py::ssize_t ray=0; ray<count && !timed_out; ++ray) {
        const double* origin=starts+3*ray; double direction[3];
        for (int axis=0; axis<3; ++axis) direction[axis]=ends[3*ray+axis]-origin[axis];
        const double length=std::sqrt(dot_norm(direction,direction));
        needs_oracle |= near_boundary(length,1e-12,length);
        if (!(length>1e-12) || !std::isfinite(length)) continue;
        for (double& value:direction) value/=length;
        layer.clear(); layer.push_back(0);
        while (!layer.empty() && !timed_out) {
          next.clear();
          for (const auto node:layer) {
            if ((work++ & 127U)==0 && expired(deadline)) {timed_out=true; break;}
            double entry=0., exit=kInf; bool valid=true;
            for (int axis=0; axis<3; ++axis) {
              const double lo=lo_[3*node+axis], hi=hi_[3*node+axis];
              needs_oracle |= near_boundary(std::abs(direction[axis]),1e-12,std::abs(direction[axis]));
              if (std::abs(direction[axis])<1e-12) {
                valid &= origin[axis]>=lo-1e-9 && origin[axis]<=hi+1e-9;
              } else {
                const double a=(lo-origin[axis])/direction[axis];
                const double b=(hi-origin[axis])/direction[axis];
                entry=std::max(entry,std::min(a,b)); exit=std::min(exit,std::max(a,b));
              }
            }
            needs_oracle |= near_boundary(exit,entry-1e-9,std::max(std::abs(exit),std::abs(entry))) ||
                near_boundary(entry,std::min(length+padding,output[ray])+1e-9,length+std::abs(padding));
            if (!valid || !(exit>=entry-1e-9) ||
                !(entry<=std::min(length+padding,output[ray])+1e-9)) continue;
            if (child_[2*node]>=0) {
              next.push_back(child_[2*node]); next.push_back(child_[2*node+1]); continue;
            }
            for (int32_t slot=span_[2*node]; slot<span_[2*node+1]; ++slot) {
              if ((work++ & 127U)==0 && expired(deadline)) {timed_out=true; break;}
              const auto triangle=index_[slot]; const double* e1=edge_+6*triangle;
              const double* e2=e1+3; double h[3]; cross(direction,e2,h);
              const double determinant=dot_einsum(e1,h);
              needs_oracle |= near_boundary(std::abs(determinant),1e-12,dot_magnitude(e1,h));
              if (!(std::abs(determinant)>1e-12)) continue;
              const double inverse=1./determinant; double delta[3],q[3];
              for (int axis=0; axis<3; ++axis)
                delta[axis]=origin[axis]-static_cast<double>(tri_[9*triangle+axis])*scale_data_[axis];
              const double u=inverse*dot_einsum(delta,h); cross(delta,e1,q);
              const double v=inverse*dot_einsum(direction,q), distance=inverse*dot_einsum(e2,q);
              const double u_scale=std::abs(inverse)*dot_magnitude(delta,h);
              const double v_scale=std::abs(inverse)*dot_magnitude(direction,q);
              const double distance_scale=std::abs(inverse)*dot_magnitude(e2,q);
              // This band is NOT a hit tolerance. A numerically marginal
              // query is recomputed by the original NumPy oracle under the
              // same absolute deadline, so no marginal hit is newly accepted.
              needs_oracle |= near_boundary(u,-1e-9,u_scale) || near_boundary(v,-1e-9,v_scale) ||
                  near_boundary(u+v,1.+1e-9,u_scale+v_scale) ||
                  near_boundary(distance,0.,distance_scale) ||
                  near_boundary(distance,length+padding+1e-9,distance_scale+length+std::abs(padding));
              if (u>=-1e-9 && v>=-1e-9 && u+v<=1.+1e-9 && distance>=0. &&
                  distance<=length+padding+1e-9)
                output[ray]=std::min(output[ray],distance);
            }
          }
          layer.swap(next);
        }
      }
      timed_out = timed_out || expired(deadline);
      if (timed_out) std::fill(output,output+count,kInf);
    }
    if (checked) return py::make_tuple(std::move(result),timed_out,needs_oracle);
    return py::make_tuple(std::move(result),timed_out);
  }

 private:
  static double dot_norm(const double* a,const double* b) noexcept {
    return (a[0]*b[0]+a[1]*b[1])+a[2]*b[2];
  }
  static double dot_einsum(const double* a,const double* b) noexcept {
    // NumPy's three-column einsum kernel adds its two SIMD lanes as
    // (product[0]+product[2])+product[1], unlike the norm reduction above.
    // This order matters at the unchanged barycentric epsilon boundary.
    return (a[0]*b[0]+a[2]*b[2])+a[1]*b[1];
  }
  static double dot_magnitude(const double* a,const double* b) noexcept {
    return std::abs(a[0]*b[0])+std::abs(a[1]*b[1])+std::abs(a[2]*b[2]);
  }
  static bool near_boundary(double value,double boundary,double magnitude) noexcept {
    if (!std::isfinite(value) || !std::isfinite(boundary) || !std::isfinite(magnitude)) return false;
    const double scale=std::max({std::abs(value),std::abs(boundary),std::abs(magnitude),1e-300});
    return std::abs(value-boundary)<=64.*std::numeric_limits<double>::epsilon()*scale;
  }
  static void cross(const double* a,const double* b,double* out) noexcept {
    out[0]=a[1]*b[2]-a[2]*b[1]; out[1]=a[2]*b[0]-a[0]*b[2]; out[2]=a[0]*b[1]-a[1]*b[0];
  }
  Array<float> triangles_; Array<double> edges_,scale_,lower_,upper_;
  Array<int32_t> children_,spans_; Array<py::ssize_t> indices_;
  const float* tri_{}; const double *edge_{},*scale_data_{},*lo_{},*hi_{};
  const int32_t *child_{},*span_{}; const py::ssize_t* index_{};
  py::ssize_t triangle_count_{},node_count_{};
};

PYBIND11_MODULE(_mesh_ray_native,module) {
  module.doc()="Exact immutable mesh BVH queries with the original monotonic deadline";
  module.attr("clock_id")="CLOCK_MONOTONIC";
  module.attr("schema")=1;
  py::class_<Mesh>(module,"Mesh")
    .def(py::init<Array<float>,Array<double>,Array<double>,Array<double>,Array<double>,
                  Array<int32_t>,Array<int32_t>,Array<py::ssize_t>>(),
      py::arg("triangles").noconvert(),py::arg("edges").noconvert(),py::arg("scale").noconvert(),
      py::arg("lower").noconvert(),py::arg("upper").noconvert(),py::arg("children").noconvert(),
      py::arg("spans").noconvert(),py::arg("indices").noconvert())
    .def("surface_candidates",&Mesh::surface_candidates,py::arg("points").noconvert(),
      py::arg("padding"),py::arg("deadline"))
    .def("first_intersections",&Mesh::first_intersections,py::arg("origins").noconvert(),
      py::arg("endpoints").noconvert(),py::arg("padding"),py::arg("deadline"),py::arg("checked")=false);
}
