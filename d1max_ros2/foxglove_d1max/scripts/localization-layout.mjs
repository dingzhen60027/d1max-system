// Preserve every split, camera and non-map panel. No controls or URDF are added.
export function localizationLayout(current){
 const data=structuredClone(current),map=data.configById['3D!d1map'];
 if(!map)throw Error('Existing PCD window is required; refusing to rearrange layout');
 map.followTf='d1max_loc_map';map.followMode='follow-pose';map.foxglovePanelTitle='PCD · LOCALIZATION';
 map.topics={
  '/d1max/localization/map_cloud':{visible:true,colorMode:'flat',flatColor:'#8294a5',pointSize:1.5,decayTime:0},
  '/d1max/localization/scan_leveled':{visible:true,colorMode:'flat',flatColor:'#38e8c6',pointSize:3,decayTime:0},
  '/d1max/localization/scan_initial_preview':{visible:true,colorMode:'flat',flatColor:'#f59e0b',pointSize:2,decayTime:0.5},
  '/d1max/localization/pose':{visible:true,color:'#fbbf24',scale:1},
  '/d1max/localization/trajectory':{visible:true,color:'#fbbf24',lineWidth:0.03},
 };
 delete map.publish;
 const status=data.configById['d1max-console.D1 状态监控!d1status'];
 if(status?.config)status.config.localizationTopic='/d1max/localization/status';
 return data;
}
