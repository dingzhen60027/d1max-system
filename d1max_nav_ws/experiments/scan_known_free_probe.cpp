#include <plan_env/raycast.h>
#include <plan_env/voxel_collision.hpp>
#include <plan_env/observed_ray.hpp>
#include <fstream>
#include <iostream>
#include <set>
#include <array>
#include <cstring>
using Cell=std::array<int,3>;
Cell cell(const Eigen::Vector3d&p){return {int(std::floor(p.x()/.08)),int(std::floor(p.y()/.08)),int(std::floor(p.z()/.08))};}
int main(int argc,char**argv){
 std::ifstream f(argv[1],std::ios::binary); char hdr[12];f.read(hdr,10);int n=(unsigned char)hdr[8]+256*(unsigned char)hdr[9];std::string h(n,' ');f.read(h.data(),n);std::vector<float> pts;float x;while(f.read(reinterpret_cast<char*>(&x),4))pts.push_back(x);
 for(int mode=0;mode<3;++mode){std::set<Cell> ends,known,traversed;known.insert({0,0,6});std::size_t steps=0;
 for(size_t j=0;j<pts.size();j+=3){Eigen::Vector3d p(pts[j],pts[j+1],pts[j+2]),o(0,0,.55);Cell end=cell(p);known.insert(end);if(!ends.insert(end).second&&mode==0)continue;
 if(mode==2){std::size_t budget=2048;bool ok=scan_planner::visitObservedRay(o,p,.08,false,budget,[&](const Eigen::Vector3i &v){known.insert({v.x(),v.y(),v.z()});++steps;});if(!ok){std::cout<<"FAIL point="<<j/3<<" end="<<p.transpose()<<" budget="<<budget<<"\n";return 2;}}
 else {RayCaster ray;ray.setInput(p/.08,o/.08);Eigen::Vector3d pt;while(ray.step(pt)){Cell c{int(pt.x()),int(pt.y()),int(pt.z())};++steps;known.insert(c);if(!traversed.insert(c).second&&mode==0)break;}}
 }
 std::cout<<"mode="<<mode<<" known="<<known.size()<<" steps="<<steps<<"\n";
 auto kernel=scan_planner::conservativeCylinderInflation(.08,.29,.15,.45);
 for(double bodyx: {0.,.5,1.,2.,3.,4.}){int unknown=0;Cell first{};for(double off:{-.2,.2}){auto id=cell({bodyx+off,0,.55});for(auto&v:kernel){Cell c{id[0]-v.x(),id[1]-v.y(),id[2]-v.z()};if(!known.count(c)){if(!unknown)first=c;++unknown;}}}std::cout<<" x="<<bodyx<<" unknown="<<unknown<<" first=("<<(first[0]+.5)*.08<<","<<(first[1]+.5)*.08<<","<<(first[2]+.5)*.08<<")\n";}
 }
}
