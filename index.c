struct AABB
{
  Vec2 min;
  Vec2 max;
};
bool AABBvsAABB( AABB a, AABB b )
{
  // Exit with no intersection if found separated along an axis
  if(a.max.x < b.min.x or a.min.x > b.max.x) return false
  if(a.max.y < b.min.y or a.min.y > b.max.y) return false
 
  // No separating axis found, therefor there is at least one overlapping axis
  return true
}
struct Circle
{
  float radius
  Vec position
};
float Distance( Vec2 a, Vec2 b )
{
  return sqrt( (a.x - b.x)^2 + (a.y - b.y)^2 )
}
 
bool CirclevsCircleUnoptimized( Circle a, Circle b )
{
  float r = a.radius + b.radius
  return r < Distance( a.position, b.position )
}
 
bool CirclevsCircleOptimized( Circle a, Circle b )
{
  float r = a.radius + b.radius
  r *= r
  return r < (a.x + b.x)^2 + (a.y + b.y)^2
}void ResolveCollision( Object A, Object B )
{
  // Calculate relative velocity
  Vec2 rv = B.velocity - A.velocity
 
  // Calculate relative velocity in terms of the normal direction
  float velAlongNormal = DotProduct( rv, normal )
 
  // Do not resolve if velocities are separating
  if(velAlongNormal > 0)
    return;
 
  // Calculate restitution
  float e = min( A.restitution, B.restitution)
 
  // Calculate impulse scalar
  float j = -(1 + e) * velAlongNormal
  j /= 1 / A.mass + 1 / B.mass
 
  // Apply impulse
  Vec2 impulse = j * normal
  A.velocity -= 1 / A.mass * impulse
  B.velocity += 1 / B.mass * impulse
}
A.inv_mass = 1 / A.mass
float mass_sum = A.mass + B.mass
float ratio = A.mass / mass_sum
A.velocity -= ratio * impulse
 
ratio = B.mass / mass_sum
B.velocity += ratio * impulse
if(A.mass == 0)
  A.inv_mass = 0
else
  A.inv_mass = 1 / A.mass
  void PositionalCorrection( Object A, Object B )
{
  const float percent = 0.2 // usually 20% to 80%
  Vec2 correction = penetrationDepth / (A.inv_mass + B.inv_mass)) * percent * n
  A.position -= A.inv_mass * correction
  B.position += B.inv_mass * correction
}
void PositionalCorrection( Object A, Object B )
{
  const float percent = 0.2 // usually 20% to 80%
  const float slop = 0.01 // usually 0.01 to 0.1
  Vec2 correction = max( penetration - k_slop, 0.0f ) / (A.inv_mass + B.inv_mass)) * percent * n
  A.position -= A.inv_mass * correction
  B.position += B.inv_mass * correction
}
struct Manifold
{
  Object *A;
  Object *B;
  float penetration;
  Vec2 normal;
};
bool CirclevsCircle( Manifold *m )
{
  // Setup a couple pointers to each object
  Object *A = m->A;
  Object *B = m->B;
 
  // Vector from A to B
  Vec2 n = B->pos - A->pos
 
  float r = A->radius + B->radius
  r *= r
 
  if(n.LengthSquared( ) > r)
    return false
 
  // Circles have collided, now compute manifold
  float d = n.Length( ) // perform actual sqrt
 
  // If distance between circles is not zero
  if(d != 0)
  {
    // Distance is difference between radius and distance
    m->penetration = r - d
 
    // Utilize our d since we performed sqrt on it already within Length( )
    // Points from A to B, and is a unit vector
    c->normal = t / d
    return true
  }
 
  // Circles are on same position
  else
  {
    // Choose random (but consistent) values
    c->penetration = A->radius
    c->normal = Vec( 1, 0 )
    return true
  }
}
bool AABBvsAABB( Manifold *m )
{
  // Setup a couple pointers to each object
  Object *A = m->A
  Object *B = m->B
  
  // Vector from A to B
  Vec2 n = B->pos - A->pos
  
  AABB abox = A->aabb
  AABB bbox = B->aabb
  
  // Calculate half extents along x axis for each object
  float a_extent = (abox.max.x - abox.min.x) / 2
  float b_extent = (bbox.max.x - bbox.min.x) / 2
  
  // Calculate overlap on x axis
  float x_overlap = a_extent + b_extent - abs( n.x )
  
  // SAT test on x axis
  if(x_overlap > 0)
  {
    // Calculate half extents along x axis for each object
    float a_extent = (abox.max.y - abox.min.y) / 2
    float b_extent = (bbox.max.y - bbox.min.y) / 2
  
    // Calculate overlap on y axis
    float y_overlap = a_extent + b_extent - abs( n.y )
  
    // SAT test on y axis
    if(y_overlap > 0)
    {
      // Find out which axis is axis of least penetration
      if(x_overlap > y_overlap)
      {
        // Point towards B knowing that n points from A to B
        if(n.x < 0)
          m->normal = Vec2( -1, 0 )
        else
          m->normal = Vec2( 0, 0 )
        m->penetration = x_overlap
        return true
      }
      else
      {
        // Point toward B knowing that n points from A to B
        if(n.y < 0)
          m->normal = Vec2( 0, -1 )
        else
          m->normal = Vec2( 0, 1 )
        m->penetration = y_overlap
        return true
      }
    }
  }
}
bool AABBvsCircle( Manifold *m )
{
  // Setup a couple pointers to each object
  Object *A = m->A
  Object *B = m->B
 
  // Vector from A to B
  Vec2 n = B->pos - A->pos
 
  // Closest point on A to center of B
  Vec2 closest = n
 
  // Calculate half extents along each axis
  float x_extent = (A->aabb.max.x - A->aabb.min.x) / 2
  float y_extent = (A->aabb.max.y - A->aabb.min.y) / 2
 
  // Clamp point to edges of the AABB
  closest.x = Clamp( -x_extent, x_extent, closest.x )
  closest.y = Clamp( -y_extent, y_extent, closest.y )
 
  bool inside = false
 
  // Circle is inside the AABB, so we need to clamp the circle's center
  // to the closest edge
  if(n == closest)
  {
    inside = true
 
    // Find closest axis
    if(abs( n.x ) > abs( n.y ))
    {
      // Clamp to closest extent
      if(closest.x > 0)
        closest.x = x_extent
      else
        closest.x = -x_extent
    }
 
    // y axis is shorter
    else
    {
      // Clamp to closest extent
      if(closest.y > 0)
        closest.y = y_extent
      else
        closest.y = -y_extent
    }
  }
 
  Vec2 normal = n - closest
  real d = normal.LengthSquared( )
  real r = B->radius
 
  // Early out of the radius is shorter than distance to closest point and
  // Circle not inside the AABB
  if(d > r * r && !inside)
    return false
 
  // Avoided sqrt until we needed
  d = sqrt( d )
 
  // Collision normal needs to be flipped to point outside if circle was
  // inside the AABB
  if(inside)
  {
    m->normal = -n
    m->penetration = r - d
  }
  else
  {
    m->normal = n
    m->penetration = r - d
  }
 
  return true
}
