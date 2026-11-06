import assert from 'node:assert/strict'
import test from 'node:test'
import { readWorkspaceRoute } from '../src/workspace-route.js'

const groups={planning:[{id:'live'}], '2d':[{id:'versions'},{id:'build'},{id:'archived'}]}
test('old navigation bookmarks all enter the unique BT runtime',()=>{
 for(const section of ['nav2','navigation','localization']) {
  const route=readWorkspaceRoute(`#/2d/${section}?version=historical`,groups)
  assert.equal(route.mode,'planning'); assert.equal(route.section,'live')
  assert.equal(route.query.get('version'),'historical')
 }
})
test('map editing and current navigation links preserve their routes',()=>{
 for(const section of ['versions','build','archived']) {
  assert.equal(readWorkspaceRoute(`#/2d/${section}`,groups).section,section)
 }
 assert.equal(readWorkspaceRoute('#/planning',groups).section,'live')
 assert.equal(readWorkspaceRoute('#/unknown',groups).mode,'home')
})
