@[Link("ferox")]
lib Ferox
  # Header
  GEOMETRY_MAX_VERTEX_COUNT = 8
  GEOMETRY_PIXELS_PER_UNIT = 16.0_f32
  WORLD_BAUMGARTE_FACTOR = 0.24_f32
  WORLD_DEFAULT_GRAVITY = Vector2.new(x: 0.0_f32, y: 9.8_f32)
  WORLD_ITERATION_COUNT = 10
  WORLD_MAX_OBJECT_COUNT = 4096

  {% if @top_level.has_constant? :Raylib %}
    alias AABB = Raylib::Rectangle
    alias Vector2  = Raylib::Vector2
    alias Color = Raylib::Color
  {% else %}
    struct Vector2
      x : LibC::Float
      y : LibC::Float
    end

    struct AABB
      x : LibC::Float
      y : LibC::Float
      width : LibC::Float
      height : LibC::Float
    end

    struct Color
      r : LibC::UChar
      g : LibC::UChar
      b : LibC::UChar
      a : LibC::UChar
    end
  {% end %}

  alias HashQueryFunc = Proc(LibC::Int, Void*, Bool)

  struct CollisionCache
    normal_mass : LibC::Float
    normal_scalar : LibC::Float
    tangent_mass : LibC::Float
    tangent_scalar : LibC::Float
  end

  struct CollisionContact
    id : LibC::Int
    point : Vector2
    depth : LibC::Float
    cache : CollisionCache
  end

  struct Collision
    friction : LibC::Float
    restitution : LibC::Float
    direction : Vector2
    contacts : StaticArray(CollisionContact, 2)
    count : LibC::Int
  end

  struct Ray
    origin : Vector2
    direction : Vector2
    max_distance : LibC::Float
  end

  struct RaycastHit
    body : Body*
    point : Vector2
    normal : Vector2
    distance : LibC::Float
    inside : Bool
  end

  enum ShapeType
    Unknown
    Circle
    Polygon
  end

  struct Material
    density : LibC::Float
    friction : LibC::Float
    restitution : LibC::Float
  end

  alias VerticesData = StaticArray(Vector2, GEOMETRY_MAX_VERTEX_COUNT)
  
  struct Vertices
    data : VerticesData
    count : LibC::Int
  end

  enum BodyType
    Unknown
    Static
    Kinematic
    Dynamic
  end

  enum BodyFlag
    None            = 0
    InfiniteMass    = 1
    InfiniteInertia = 2
  end

  alias BodyFlags = LibC::UChar

  struct TransformRotation
    sin : LibC::Float
    cos : LibC::Float 
  end

  struct Transform
    position : Vector2
    rotation : TransformRotation
    angle : LibC::Float
  end

  struct BodyPair
    first : Body*
    second : Body*
  end

  alias CollisionEventFunc = Proc(BodyPair, Collision*, Bool)

  struct CollisionHandler
    pre_step : CollisionEventFunc
    post_step : CollisionEventFunc
  end

  alias RaycastQueryFunc = Proc(RaycastHit, Void)

  # Broadphase
  fun create_spatial_hash = frCreateSpatialHash(cell_size : LibC::Float) : SpatialHash*
  fun release_spatial_hash = frReleaseSpatialHash(hash : SpatialHash*)
  fun clear_spatial_hash = frClearSpatialHash(hash : SpatialHash*)
  fun get_spacial_hash_cell_size = frGetSpatialHashCellSize(hash : SpatialHash*) : LibC::Float
  fun insert_to_spatial_hash = frInsertToSpatialHash(hash : SpatialHash*, key : AABB, value : LibC::Int)
  fun query_spatial_hash = frQuerySpatialHash(hash : SpatialHash*, aabb : AABB, func : HashQueryFunc, ctx : Void*)

  # Collision
  fun compute_collision  = frComputeCollision(s1 : Shape*, tx1 : Transform, s2 : Shape*, tx2 : Transform, collision : Collision*) : Bool
  fun compute_raycast = frComputeRaycast(body : Body*, ray : Ray, raycast_hit : RaycastHit*)

  # Geometry
  fun create_circle = frCreateCircle(material : Material, radius : LibC::Float) : Shape*
  fun create_rectangle = frCreateRectangle(material : Material, width : LibC::Float, height : LibC::Float) : Shape*
  fun create_polygon = frCreatePolygon(material : Material, vertices : Vertices*) : Shape*
  fun release_shape = frReleaseShape(s : Shape*)
  fun get_shape_type = frGetShapeType(s : Shape*) : ShapeType
  fun get_shape_material = frGetShapeMaterial(s : Shape*) : Material
  fun get_shape_density = frGetShapeDensity(s : Shape*) : LibC::Float
  fun get_shape_friction = frGetShapeFriction(s : Shape*) : LibC::Float
  fun get_shape_restituion = frGetShapeRestitution(s : Shape*) : LibC::Float
  fun get_shape_area = frGetShapeArea(s : Shape*) : LibC::Float
  fun get_shape_mass = frGetShapeMass(s : Shape*) : LibC::Float
  fun get_shape_intertia = frGetShapeInertia(s : Shape*) : LibC::Float
  fun get_shape_aabb = frGetShapeAABB(s : Shape*, tx : Transform) : AABB
  fun get_circle_radius = frGetCircleRadius(s : Shape*) : LibC::Float
  fun get_polygon_vertex = frGetPolygonVertex(s : Shape*, index : LibC::Int) : Vector2
  fun get_polygon_vertices = frGetPolygonVertices(s : Shape*) : Vertices*
  fun get_polygon_normal = frGetPolygonNormal(s : Shape*, index : LibC::Int) : Vector2
  fun get_polygon_normals = frGetPolygonNormals(s : Shape*) : Vertices*
  fun set_shape_type = frSetShapeType(s : Shape*, type : ShapeType)
  fun set_shape_material = frSetShapeMaterial(s : Shape*, material : Material)
  fun set_shape_density = frSetShapeDensity(s : Shape*, density : LibC::Float)
  fun set_shape_friction = frSetShapeFriction(s : Shape*, friction : LibC::Float)
  fun set_shape_restitution = frSetShapeRestitution(s : Shape*, restitution : LibC::Float)
  fun set_circle_radius = frSetCircleRadius(s : Shape*, radius : LibC::Float)
  fun set_rectangle_dimensions = frSetRectangleDimensions(s : Shape*, width : LibC::Float, height : LibC::Float)
  fun set_polygon_vertices = frSetPolygonVertices(s : Shape*, vertices : Vertices*)

  # Rigid Body
  fun create_body = frCreateBody(type : BodyType, position : Vector2) : Body*
  fun create_body_from_shape = frCreateBodyFromShape(type : BodyType, position : Vector2, shape : Shape*) : Body*
  fun release_body = frReleaseBody(b : Body*)
  fun get_body_type = frGetBodyType(b : Body*) : BodyType
  fun get_body_flags = frGetBodyFlags(b : Body*) : BodyFlags
  fun get_body_shape = frGetBodyShape(b : Body*) : Shape*
  fun get_body_transform = frGetBodyTransform(b : Body*) : Transform
  fun get_body_position = frGetBodyPosition(b : Body*) : Vector2
  fun get_body_angle = frGetBodyAngle(b : Body*) : LibC::Float
  fun get_body_mass = frGetBodyMass(b : Body*) : LibC::Float
  fun get_body_inverse_mass = frGetBodyInverseMass(b : Body*) : LibC::Float
  fun get_body_intertia = frGetBodyInertia(b : Body*) : LibC::Float
  fun get_body_inverse_intertia = frGetBodyInverseInertia(b : Body*) : LibC::Float
  fun get_body_gravity_scale = frGetBodyGravityScale(b : Body*) : LibC::Float
  fun get_body_velocity = frGetBodyVelocity(b : Body*) : Vector2
  fun get_body_aabb = frGetBodyAABB(b : Body*) : AABB
  fun get_body_user_data = frGetBodyUserData(b : Body*) : Void*
  fun set_body_type = frSetBodyType(b : Body*, type : BodyType)
  fun set_body_flags = frSetBodyFlags(b : Body*, type : BodyFlags)
  fun set_body_shape = frSetBodyShape(b : Body*, shape : Shape*)
  fun set_body_transform = frSetBodyTransform(b : Body*, tx : Transform)
  fun set_body_position = frSetBodyPosition(b : Body*, position : Vector2)
  fun set_body_angle = frSetBodyAngle(b : Body*, angle : LibC::Float)
  fun set_body_gravity_scale = frSetBodyGravityScale(b : Body*, scale : LibC::Float)
  fun set_body_velocity = frSetBodyVelocity(b : Body*, v : Vector2) : Vector2
  fun set_body_user_data = frSetBodyUserData(b : Body*, data : Void*)
  fun clear_body_forces = frClearBodyForces(b : Body*)
  fun apply_force_to_body = frApplyForceToBody(b : Body*, point : Vector2, force : Vector2)
  fun apply_gravity_to_body = frApplyGravityToBody(b : Body*, g : Vector2)
  fun apply_impulse_to_body = frApplyForceToBody(b : Body*, point : Vector2, impulse : Vector2)
  fun apply_accumulated_impulses = frApplyAccumulatedImpulses(b1 : Body*, b2 : Body*, ctx : Collision*)
  fun integrate_for_body_velocity = frIntegrateForBodyVelocity(b : Body*, dt : LibC::Float)
  fun integrate_for_body_position = frIntegrateForBodyPosition(b : Body*, dt : LibC::Float)
  fun resolve_collision = frResolveCollision(b1 : Body*, b2 : Body*, ctx : Collision*, inverse_dt : LibC::Float)
  fun get_current_time = frGetCurrentTime() : LibC::Double

  # World
  fun create_world = frCreateWorld(gravity : Vector2, cellsize : LibC::Float) : World*
  fun release_world = frReleaseWorld(world : World*)
  fun clear_world = frClearWorld(world : World*)
  fun add_body_to_world = frAddBodyToWorld(world : World*, b : Body*) : Bool
  fun remove_body_from_world = frRemoveBodyFromWorld(world : World*, b : Body*) : Bool
  fun get_body_from_world = frGetBodyFromWorld(world : World*, index : LibC::Int) : Body*
  fun get_body_count_for_world = frGetBodyCountForWorld(world : World*) : LibC::Int
  fun get_world_gravity = frGetWorldGravity(world : World*) : Vector2
  fun set_world_collision_handler = frSetWorldCollisionHandler(world : World*, handler : CollisionHandler)
  fun set_world_gravity = frSetWorldGravity(world : World*, gravity : Vector2)
  fun step_world = frStepWorld(world : World*, dt : LibC::Double)
  fun update_world = frUpdateWorld(world : World*, dt : LibC::Double)
  fun computer_raycast_for_world = frComputeRaycastForWorld(world : World*, ray : Ray, func : RaycastQueryFunc)

  # Inline
  fun vec2_add = frVector2Add(v1 : Vector2, v2 : Vector2) : Vector2
  fun vec2_subtract = frVector2Subtract(v1 : Vector2, v2 : Vector2) : Vector2
  fun vec2_negate = frVector2Negate(v : Vector2) : Vector2
  fun vec2_scalar_multiply = frVector2ScalarMultiply(v1 : Vector2, value : LibC::Float) : Vector2
  fun vec2_dot = frVector2Dot(v1 : Vector2, v2 : Vector2) : LibC::Float
  fun vec2_cross = frVector2Cross(v1 : Vector2, v2 : Vector2) : LibC::Float
  fun vec2_magnitude_sqr = frVector2MagnitudeSqr(v : Vector2) : LibC::Float
  fun vec2_magnitude = frVector2Magnitude(v : Vector2) : LibC::Float
  fun vec2_distance_sqr = frVector2DistanceSqr(v1 : Vector2, v2 : Vector2) : LibC::Float
  fun vec2_distance = frVector2Distance(v1 : Vector2, v2 : Vector2)  : LibC::Float
  fun vec2_normalize = frVector2Normalize(v : Vector2) : Vector2
  fun vec2_left_normal = frVector2LeftNormal(v : Vector2) : Vector2
  fun vec2_right_normal = frVector2RightNormal(v : Vector2) : Vector2
  fun vec2_rotate = frVector2Rotate(v : Vector2, angle : LibC::Float) : Vector2
  fun vec2_rotate_tx = frVector2RotateTx(v : Vector2, tx : Transform) : Vector2
  fun vec2_transform = frVector2Transform(v : Vector2, tx : Transform) : Vector2
  fun vec2_counter_clockwise = frVector2CounterClockwise(v1 : Vector2, v2 : Vector2, v3 : Vector2) : Bool

  fun vec2_angle = frVector2Angle(v1 : Vector2, v2 : Vector2) : LibC::Float
  fun vec2_pixels_to_units = frVector2PixelsToUnits(v : Vector2) : Vector2
  fun vec2_units_to_pixels = frVector2UnitsToPixels(v : Vector2) : Vector2
  fun pixels_to_units = frPixelsToUnits(value : LibC::Float) : LibC::Float
  fun units_to_pixels = frUnitsToPixels(value : LibC::Float) : LibC::Float

  # End of Header

  # broad-phase.c
  struct SpatialHashKey
    x : LibC::Int
    y : LibC::Int
  end

  struct SpatialHashEntry
    key : SpatialHashKey
    value : LibC::Int*
  end

  struct SpatialHash
    cell_size : LibC::Float
    inverse_cell_size : LibC::Float
    query_result : LibC::Int*
    entries : SpatialHashEntry*
  end

  # collision.c
  struct Edge
    data : StaticArray(Vector2, 2)
    indexes : StaticArray(LibC::Int, 2)
    count : LibC::Int
  end

  fun clip_edge = frClipEdge(e : Edge*, v : Vector2, dot : LibC::Float) : Bool
  fun compute_collision_circles = frComputeCollisionCircles(s1 : Shape*, tx1 : Transform, s2 : Shape*, tx2 : Transform, collision : Collision*) : Bool
  fun compute_collision_circle_poly = frComputeCollisionCirclePoly(s1 : Shape*, tx1 : Transform, s2 : Shape*, tx2 : Transform, collision : Collision*) : Bool
  fun compute_collision_circle_polys = frComputeCollisionCirclePolys(s1 : Shape*, tx1 : Transform, s2 : Shape*, tx2 : Transform, collision : Collision*) : Bool
  fun compute_intersection_circle_line = frComputeIntersectionCircleLine(center : Vector2, radius : LibC::Float, origin : Vector2, direction : Vector2, lambda : LibC::Float*) : Bool
  fun compute_intersection_lines = frComputeIntersectionLines(origin1 : Vector2, direction1 : Vector2, origin2 : Vector2, direction2 : Vector2, lambda : LibC::Float*) : Bool
  fun get_contact_edge = frGetContactEdge(s : Shape*, tx : Transform, v : Vector2) : Edge
  fun get_separating_axis_index = frGetSeparatingAxisIndex(s1 : Shape*, tx1 : Transform, s2 : Shape*, tx2 : Transform, depth : LibC::Float*) : LibC::Int
  fun get_support_point_index = frGetSupportPointIndex(vertices : Vertices*, tx : Transform, v : Vector2) : LibC::Int
  
  # geometry.c

  struct Circle
    radius : LibC::Float
  end

  struct Polygon
    vertices : Vertices
    normals : Vertices
  end

  union ShapeData
    circle : Circle
    polygon : Polygon
  end

  struct Shape
    type : ShapeType
    material : Material
    area : LibC::Float
    data : ShapeData
  end

  fun jarvis_march = frJarvisMarch(input : Vertices*, output : Vertices*)

  # rigid-body.c

  struct MotionData
    mass : LibC::Float
    inverse_mass : LibC::Float
    inertia : LibC::Float
    inverse_inertia : LibC::Float
    gravity_scale : LibC::Float
    velocity : Vector2
    angular_velocity : LibC::Float
    force : Vector2
    torque : LibC::Float
  end

  struct Body
    type : BodyType
    flags : BodyFlags
    shape : Shape*
    tx : Transform
    mtn : MotionData
    aabb : AABB
    ctx : Void*
  end

  fun compute_body_mass = frComputeBodyMass(b : Body*)
  fun normalize_angle = frNormalizeAngle(angle : LibC::Float) : LibC::Float

  # world.c
  struct ContactCacheEntry
    key : BodyPair
    value : Collision
  end

  struct World
    gravity : Vector2
    bodies : Body**
    hash : SpatialHash*
    cache : ContactCacheEntry*
    handler : CollisionHandler
    accumulator : LibC::Double
    timestamp : LibC::Double
  end

  struct PreStepHashQuery
    world : World*
    body_index : LibC::Int
  end

  struct RaycastHashQueryCTX
    ray : Ray
    world : World*
    func : RaycastQueryFunc
  end

  fun pre_step_hash_query_callback = frPreStepHashQueryCallback(other_body_index : LibC::Int, ctx : Void*) : Bool
  fun raycast_hash_query_callback = frRaycastHashQueryCallback(body_index : LibC::Int, ctx : Void*) : Bool
  fun pre_step_world = frPreStepWorld(w : World*)
  fun post_step_world = frPostStepWorld(w : World*)
end