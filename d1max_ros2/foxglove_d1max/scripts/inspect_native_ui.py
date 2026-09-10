import pyatspi
desktop=pyatspi.Registry.getDesktop(0)
def walk(item, depth=0):
    if depth>12:
        return
    role=item.getRoleName()
    if ('menu' in role or role in ('dialog','text','entry')) and item.getState().contains(pyatspi.STATE_SHOWING):
        print(' '*depth,role,repr(item.name))
    for child in item:
        if child is not None:
            walk(child,depth+1)
for app in desktop:
    if app and 'foxglove' in app.name.lower():
        print('APP',app.name)
        walk(app)
