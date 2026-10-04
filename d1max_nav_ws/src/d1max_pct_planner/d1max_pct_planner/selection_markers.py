"""Stable-ID support witnesses. They display selection evidence, not permission."""
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def selection_evidence_markers(role, xyz, evidence, header, first_id):
    result = []
    for offset, kind in enumerate((Marker.LINE_STRIP, Marker.LINE_LIST, Marker.TEXT_VIEW_FACING)):
        marker = Marker(header=header, ns='selected_support', id=first_id+offset, type=kind)
        marker.pose.orientation.w = 1.
        marker.action = Marker.ADD if xyz is not None else Marker.DELETE
        marker.scale.x = .025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            (.2, 1., .35, 1.) if evidence['valid'] else (1., .2, .15, 1.))
        if xyz is not None:
            x, y, z = map(float, xyz)
            ground = evidence.get('ground_z')
            if offset < 2 and ground is None:
                marker.action = Marker.DELETE
            elif offset == 0:
                marker.points = [Point(x=x+.25*math.cos(i*math.pi/24),
                    y=y+.25*math.sin(i*math.pi/24), z=ground+.03) for i in range(49)]
            elif offset == 1:
                marker.points = [Point(x=x, y=y, z=z), Point(x=x, y=y, z=ground+.03)]
            else:
                floor = {'floor1': '一楼', 'floor2': '二楼'}.get(evidence.get('floor_id'), '楼梯/未定层')
                marker.pose.position = Point(x=x, y=y, z=z+.5)
                marker.scale.z = .28
                state = '地面有效' if evidence['valid'] else '无效选点'
                error = evidence.get('height_error_m')
                detail = f' · ΔZ {error:+.2f}m' if error is not None else ' · 无地面证据'
                marker.text = f'{"起点" if role == "start" else "终点"} · {floor} · {state}{detail}'
        result.append(marker)
    return result
