# require "raylib-cr"
require "ferox-cr"
# alias R = Raylib
alias F = Ferox


# module Basic
#   TARGET_FPS = 60
#   SCREEN_WIDTH = 800
#   SCREEN_HEIGHT = 600
#   CELL_SIZE = 4.0_f32
#   DELTA_TIME = 1.0_f32 / TARGET_FPS

#   def self.draw_body_lines(b : Pointer(F::Body), thick : Float, color : R::Color)
#     return if b.null? || thick < 0

#     shape = F.get_body_shape(b)
#     tx = F.get_body_transform(b)
#     position = F.vec2_units_to_pixels(F.get_body_position(b))

#     case F.get_shape_type(shape)
#     when F::ShapeType::Circle
#       R.draw_ring(
#         position,
#         F.units_to_pixels(F.get_circle_radius(shape)) - thick,
#         F.units_to_pixels(F.get_circle_radius(shape)),
#         0,
#         360,
#         16,
#         color
#       )
#     when F::ShapeType::Polygon
#       v = F.get_polygon_vertices(shape)
#       v_count = v.value.count
#       v_count.times do |i|
#         j = i-1
#         j = v_count-1 if j < 0

#         v1 = F.vec2_transform(v.value.data[j], tx)
#         v2 = F.vec2_transform(v.value.data[i], tx)

#         v1 = F.vec2_units_to_pixels(v1)
#         v2 = F.vec2_units_to_pixels(v2)

#         R.draw_line_ex(v1, v2, thick, color)
#       end
#     end 

#     R.draw_circle_v(position, 2.0_f32, color)
#   end

#   def self.run
#     bounds = R::Rectangle.new(
#       x: 0,
#       y: 0,
#       width: SCREEN_WIDTH,
#       height: SCREEN_HEIGHT
#     )
    
#     world = F.create_world(
#       F.vec2_scalar_multiply(F::WORLD_DEFAULT_GRAVITY, 4.0_f32),
#       CELL_SIZE
#     )

#     ground = F.create_body_from_shape(
#       F::BodyType::Static,
#       F.vec2_pixels_to_units(
#         R::Vector2.new(
#           x: 0.5_f32 * SCREEN_WIDTH,
#           y: 0.85_f32 * SCREEN_HEIGHT
#         )
#       ),
#       F.create_rectangle(
#         F::Material.new(
#           density: 1.25_f32,
#           friction: 0.5_f32
#         ),
#         F.pixels_to_units(0.75_f32 * SCREEN_WIDTH),
#         F.pixels_to_units(0.1_f32 * SCREEN_HEIGHT)
#       )
#     )
    

#     F.add_body_to_world(world, ground)

#     box = F.create_body_from_shape(
#       F::BodyType::Dynamic,
#       F.vec2_pixels_to_units(
#         R::Vector2.new(
#           x: 0.5_f32 * SCREEN_WIDTH,
#           y: 0.35_f32 * SCREEN_HEIGHT
#         )
#       ),
#       F.create_rectangle(
#         F::Material.new(
#           density: 1.00_f32,
#           friction: 0.35_f32
#         ),
#         F.pixels_to_units(45.0_f32),
#         F.pixels_to_units(45.0_f32)
#       )
#     )

#     F.add_body_to_world(world, box)

#     until R.close_window?
#       F.update_world(world, DELTA_TIME)
#       R.begin_drawing
#       R.clear_background(R::BLACK)
#       draw_body_lines(ground, 1.0_f32, R::BLUE)
#       draw_body_lines(box, 1.0_f32, R::RED)

#       R.end_drawing
#     end

#     F.release_shape(F.get_body_shape(ground))
#     F.release_shape(F.get_body_shape(box))
#     F.release_world(world)
#   end
# end

# Basic.run
puts F.vec2_add(F::Vector2.new, F::Vector2.new)
puts F.vec2_subtract(F::Vector2.new, F::Vector2.new)
puts F.vec2_negate(F::Vector2.new)
puts F.vec2_scalar_multiply(F::Vector2.new, 3.0_f32)
puts F.vec2_cross(F::Vector2.new, F::Vector2.new)
puts F.vec2_dot(F::Vector2.new, F::Vector2.new)
puts F.vec2_magnitude_sqr(F::Vector2.new)
puts F.vec2_magnitude(F::Vector2.new)
puts F.vec2_angle(F::Vector2.new, F::Vector2.new)
puts F.vec2_normalize(F::Vector2.new)
puts F.vec2_left_normal(F::Vector2.new)
puts F.vec2_right_normal(F::Vector2.new)
puts F.vec2_rotate(F::Vector2.new, 3.0_f32)
puts F.vec2_rotate_tx(F::Vector2.new, F::Transform.new)
puts F.vec2_transform(F::Vector2.new, F::Transform.new)
puts F.vec2_ccw(F::Vector2.new,F::Vector2.new,F::Vector2.new)
puts F.vec2_units_to_pixels(F::Vector2.new)
puts F.vec2_pixels_to_units(F::Vector2.new)
puts F.units_to_pixels(10)
puts F.pixels_to_units(10)
