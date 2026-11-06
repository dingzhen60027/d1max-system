# Parametric planning prior, NEVER a literal observation or motion permission.
# pose maps the orthonormal model axes (u, v, up) into SupportReference.frame_id.
# The finite convex polygon is expressed in model (u,v), with z=0.
string surface_id
string segment_id
string kind
geometry_msgs/Pose pose
geometry_msgs/Point[] polygon
# In model coordinates: support w = grade*u + intercept. Body height is NOT
# included. continuous_support_envelope describes a ramp approximation, not
# measured step dimensions. landing has grade=0 in its fitted local basis.
float64 support_grade
float64 support_intercept_m
# Finite model-up association band around this support, to distinguish
# overlapping floors. Geometry domain only; not a motion/capability threshold.
float64 height_association_m
float64 contact_tolerance_m
# Reserved for a future measured standard_steps contract; currently must be
# empty. The current runtime accepts only envelopes/landings and explicitly
# rejects standard_steps rather than silently treating them as a ramp.
float64[] tread_stations_m
float64[] tread_heights_m
