// Historical navigation bookmarks enter the same BT-owned runtime as /planning.
export function readWorkspaceRoute(hash, groups) {
 const [path, query = ''] = (hash.replace(/^#/, '') || '/').split('?')
 const [, mode, section] = path.split('/')
 if (mode === '2d' && ['nav2', 'navigation', 'localization'].includes(section)) {
  return {mode: 'planning', section: 'live', query: new URLSearchParams(query)}
 }
 if (!groups[mode]) return {mode: 'home', section: '', query: new URLSearchParams()}
 return {mode, section: groups[mode].some(v => v.id === section) ? section : groups[mode][0].id,
  query: new URLSearchParams(query)}
}
