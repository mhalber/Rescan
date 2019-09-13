
// TODO(maciej): Look at randy gauls tinyc2 to see how he deals with fat_factor
#ifndef INTERSECTION_H
#define INTERSECTION_H

// Main function
float      isect_get_overlap_factor( const rs_pointcloud_t* pc_a, const msh_mat4_t pose_a,
                                     const rs_pointcloud_t* pc_b, const msh_mat4_t pose_b,
                                     const float voxel_size, const int voxelize_inside,
                                     const int normalize_by_smaller );

// Helper types and functions, made public for easy visualization 
typedef enum isect_occupancy_type
{
 ISECT_FREE_SPACE = 0,
 ISECT_BOUNDARY   = 1,
 ISECT_INSIDE     = 2,
 ISECT_OVERLAP    = 3
} occupancy_t;

typedef struct isect_grid3d
{
  int32_t x_res;
  int32_t y_res;
  int32_t z_res;
  int32_t n_cells;
  float voxel_size;
  msh_bbox_t bbox;
  uint8_t *data;
  float fat_factor;
} isect_grid3d_t;

void       isect_grid3d_init( isect_grid3d_t* grd, const msh_bbox_t* bbox, const float voxel_size );
void       isect_grid3d_term( isect_grid3d_t* grd );
uint8_t*    isect_grid3d_cell( const isect_grid3d_t* grd, int32_t x, int32_t y, int32_t z );
uint8_t*    isect_grid3d_cell_from_world_space( const isect_grid3d_t* grd, msh_vec3_t pt );


msh_bbox_t isect_get_transformed_aabbox( const rs_pointcloud_t* pointcloud, 
                                         const msh_mat4_t pose );
int        isect_compute_occupancy_grid( const rs_pointcloud_t* pc, 
                                         const msh_mat4_t pose, 
                                         isect_grid3d_t* grd );
int        isect_compute_overlap_grid( const isect_grid3d_t* grd_a, const isect_grid3d_t* grd_b,
                                       isect_grid3d_t* overlap_grd );

int32_t    isect_compute_boundary_grid( const rs_pointcloud_t* pc,
                                        const msh_mat4_t pose,
                                        isect_grid3d_t* grd );


#endif /* INTERSECTION_H */



#ifdef INTERSECTION_IMPLEMENTATION

void
isect_grid3d_init( isect_grid3d_t* grd, const msh_bbox_t* bbox, const float voxel_size )
{
  grd->fat_factor = 0.3f;
  msh_vec3_t fat_v =  msh_vec3(grd->fat_factor, grd->fat_factor, grd->fat_factor);
  grd->bbox = *bbox;
  grd->bbox.min_p = msh_vec3_sub( grd->bbox.min_p, fat_v );
  grd->bbox.max_p = msh_vec3_add( grd->bbox.max_p, fat_v );
  
  grd->x_res = (int32_t)ceilf(mshgeo_bbox_width(&grd->bbox) / voxel_size) + 1;
  grd->y_res = (int32_t)ceilf(mshgeo_bbox_height(&grd->bbox) / voxel_size) + 1;
  grd->z_res = (int32_t)ceilf(mshgeo_bbox_depth(&grd->bbox) / voxel_size) + 1;
  grd->voxel_size = voxel_size;
  grd->n_cells = grd->x_res * grd->y_res * grd->z_res;
  
  grd->data = (uint8_t*)malloc(grd->n_cells*sizeof(uint8_t));
  memset(grd->data, 0, sizeof(uint8_t)*grd->n_cells);
}

void
isect_grid3d_term( isect_grid3d_t* grd )
{
  free(grd->data);
  grd->x_res      = 0;
  grd->y_res      = 0;
  grd->z_res      = 0;
  grd->n_cells    = 0;
  grd->voxel_size = 0;
  grd->data       = NULL;
  mshgeo_bbox_reset(&grd->bbox);
}

uint8_t* 
isect_grid3d_cell( const isect_grid3d_t* grd, int32_t x, int32_t y, int32_t z )
{
  return &(grd->data[(y*grd->x_res*grd->z_res) + (z*grd->x_res) + x]);
}

uint8_t* 
isect_grid3d_cell_from_world_space( const isect_grid3d_t* grd, msh_vec3_t pt )
{
  msh_vec3_t origin = grd->bbox.min_p;
  float inv_voxel_size = 1.0f / grd->voxel_size;
  int32_t x = (int32_t)floorf( (pt.x - origin.x) * inv_voxel_size );
  int32_t y = (int32_t)floorf( (pt.y - origin.y) * inv_voxel_size );
  int32_t z = (int32_t)floorf( (pt.z - origin.z) * inv_voxel_size );
  if( (x < 0 || x >= grd->x_res) ||
      (y < 0 || y >= grd->y_res) ||
      (z < 0 || z >= grd->z_res) ) { return NULL; }

  return &(grd->data[y*grd->x_res * grd->z_res + z * grd->x_res + x]);
}

msh_bbox_t
isect_get_transformed_aabbox( const rs_pointcloud_t* pointcloud, const msh_mat4_t pose )
{
  int lvl = 3;
  msh_bbox_t bbox = mshgeo_bbox_init();
  for( size_t i = 0; i < pointcloud->n_pts[lvl]; ++i )
  {
    msh_vec3_t pos = msh_mat4_vec3_mul( pose, pointcloud->positions[lvl][i], 1 );
    mshgeo_bbox_union(&bbox, pos);
  }
  return bbox;
}


void
isect__compute_occupancy_within_slice( int8_t* slice, int x_res, int z_res, int dir )
{
  int r1_res = dir ? x_res : z_res;
  int r2_res = dir ? z_res : x_res;
  for( int r1 = 0 ; r1 < r1_res; ++r1 )
  {
    int scanline_forward[4096]  = {0};
    int scanline_backward[4096] = {0};
    
    int prev_indicator = 0;
    int indicator = 0;
    int fill = 0;
    for( int r2 = 0 ; r2 < r2_res; ++r2 )
    {
      int z = dir ? r2 : r1;
      int x = dir ? r1 : r2;
      int8_t *cell = &slice[z*x_res+x];
      indicator = *cell;
      if( indicator == ISECT_FREE_SPACE && prev_indicator == ISECT_BOUNDARY ) { fill += 1; }
      if( fill % 2 == 1 ) { scanline_forward[r2] = 1; }
      prev_indicator = indicator;
    }

    prev_indicator = 0;
    indicator = 0;
    fill = 0;
    for( int r2 = r2_res-1 ; r2 >= 0; --r2 )
    {
      int z = dir ? r2 : r1;
      int x = dir ? r1 : r2;
      int8_t *cell = &slice[z*x_res+x];
      indicator = *cell;
      if( indicator == ISECT_FREE_SPACE && prev_indicator == ISECT_BOUNDARY ) { fill += 1; }
      if( fill % 2 == 1 ) { scanline_backward[r2] = 1; }
      prev_indicator = indicator;
    }

    for( int r2 = 0 ; r2 < r2_res; ++r2 )
    {
      int z = dir ? r2 : r1;
      int x = dir ? r1 : r2;
      int8_t *cell = &slice[z*x_res+x];
      if(*cell != 1 && scanline_backward[r2] == 1 && scanline_forward[r2] == 1 ) 
      { 
        *cell = ISECT_INSIDE;
      }
    }
  }
}

int32_t
isect_compute_boundary_grid( const rs_pointcloud_t* pc,
                             const msh_mat4_t pose,
                             isect_grid3d_t* grd )
{
  int32_t lvl = 1;
  int32_t count = 0;

  // Compute boundary
  msh_vec3_t origin = grd->bbox.min_p;
  for( size_t i = 0 ; i < pc->n_pts[lvl]; ++i )
  {
    msh_vec3_t p = msh_mat4_vec3_mul( pose, pc->positions[lvl][i], 1 );
    msh_vec3_t o = msh_vec3_sub( p, origin );
    int32_t x = (int32_t)floorf( o.x / grd->voxel_size );
    int32_t y = (int32_t)floorf( o.y / grd->voxel_size );
    int32_t z = (int32_t)floorf( o.z / grd->voxel_size );

    assert(x >= 0 && x < grd->x_res );
    assert(y >= 0 && y < grd->y_res );
    assert(z >= 0 && z < grd->z_res );
    uint8_t *cell = isect_grid3d_cell( grd, x, y, z );
    *cell = (uint8_t)ISECT_BOUNDARY;
  }

  // Count cells that are boundary
  for( int32_t i = 0; i < grd->n_cells; ++i )
  {
    if( grd->data[i] == ISECT_BOUNDARY) { count++; }
  }
  
  return count;
}

int
isect_compute_occupancy_grid( const rs_pointcloud_t* pc, 
                              const msh_mat4_t pose, 
                              isect_grid3d_t* grd )
{
  int32_t lvl = 1;

  // Compute boundary
  msh_vec3_t origin = grd->bbox.min_p;
  for( size_t i = 0 ; i < pc->n_pts[lvl]; ++i )
  {
    msh_vec3_t p = msh_mat4_vec3_mul( pose, pc->positions[lvl][i], 1 );
    msh_vec3_t o = msh_vec3_sub( p, origin );
    int32_t x = (int32_t)floorf( o.x / grd->voxel_size );
    int32_t y = (int32_t)floorf( o.y / grd->voxel_size );
    int32_t z = (int32_t)floorf( o.z / grd->voxel_size );

    assert(x >= 0 && x < grd->x_res );
    assert(y >= 0 && y < grd->y_res );
    assert(z >= 0 && z < grd->z_res );
    uint8_t *cell = isect_grid3d_cell( grd, x, y, z );
    *cell = (uint8_t)ISECT_BOUNDARY;
  }

  int32_t slice_res       = grd->x_res * grd->z_res;
  int8_t* slices_data     = (int8_t*)malloc( 3 * sizeof(int8_t) * slice_res );
  int8_t* slice_x         = slices_data;
  int8_t* slice_y         = slices_data + slice_res;
  int8_t* slice_combined  = slices_data + 2*slice_res;
  memset(slice_x, 0, sizeof(int8_t) * slice_res);
  memset(slice_y, 0, sizeof(int8_t) * slice_res);
  memset(slice_combined, 0, sizeof(int8_t) * slice_res);

  for( int y = 0 ; y < grd->y_res; ++y )
  {
    memcpy( slice_x, grd->data + y*slice_res, slice_res*sizeof(int8_t) );
    memcpy( slice_y, grd->data + y*slice_res, slice_res*sizeof(int8_t) );
    isect__compute_occupancy_within_slice(slice_x, grd->x_res, grd->z_res, 0);
    isect__compute_occupancy_within_slice(slice_y, grd->x_res, grd->z_res, 1);
    for( int z = 0 ; z < grd->z_res; ++z )
    {
      for( int x = 0 ; x < grd->x_res; ++x )
      {
        int8_t val = ISECT_FREE_SPACE;
        if( slice_x[z*grd->x_res+x] == ISECT_BOUNDARY || 
            slice_y[z*grd->x_res+x] == ISECT_BOUNDARY ) val = ISECT_BOUNDARY;
        if( slice_x[z*grd->x_res+x] == ISECT_INSIDE &&
            slice_y[z*grd->x_res+x] == ISECT_INSIDE ) val = ISECT_INSIDE;
        slice_combined[z*grd->x_res+x] = val;
      } 
    }
    memcpy( grd->data + y*slice_res, slice_combined, slice_res*sizeof(int8_t) );
  }
  free(slices_data);

  int32_t count = 0;
  for(int i = 0 ; i < grd->x_res*grd->y_res*grd->z_res; ++i)
  {
    if(grd->data[i] > ISECT_FREE_SPACE) { count++; }
  }
  if (count == 0)
  {
    printf("GRID %d %d %d | %d\n", grd->x_res, grd->y_res, grd->z_res, (int32_t)pc->n_pts[lvl] );
  }
  
  return count;
}

int 
isect_compute_overlap_grid( const isect_grid3d_t* grd_a, const isect_grid3d_t* grd_b,
                            isect_grid3d_t* overlap_grd )
{
  assert( grd_a && grd_b && overlap_grd );
  assert( grd_a->x_res == grd_b->x_res );
  assert( grd_a->y_res == grd_b->y_res );
  assert( grd_a->z_res == grd_b->z_res );
  assert( overlap_grd->x_res == grd_b->x_res );
  assert( overlap_grd->y_res == grd_b->y_res );
  assert( overlap_grd->z_res == grd_b->z_res );

  int n = overlap_grd->x_res * overlap_grd->y_res * overlap_grd->z_res;
  int count = 0;
  for( int i = 0 ; i < n; ++i )
  {
    uint8_t* c_a = &(grd_a->data[i]);
    uint8_t* c_b = &(grd_b->data[i]);
    if( *c_a != ISECT_FREE_SPACE )               { overlap_grd->data[i] = *c_a; }
    if( *c_b != ISECT_FREE_SPACE )               { overlap_grd->data[i] = *c_b; }
    if( (*c_a == ISECT_BOUNDARY || *c_a == ISECT_INSIDE) &&
        (*c_b == ISECT_BOUNDARY || *c_b == ISECT_INSIDE) ) 
      { 
        overlap_grd->data[i] = ISECT_OVERLAP; 
        count++; 
      }
  }
  return count;
}

// Grids are optional / for visualization
float
isect_get_overlap_factor( const rs_pointcloud_t* pc_a, const msh_mat4_t pose_a,
                          const rs_pointcloud_t* pc_b, const msh_mat4_t pose_b,
                          const float voxel_size, const int voxelize_inside,
                          const int normalize_by_smaller )
{
  float overlap = 0.0f;
  
  // calculate tranfromed bounding boxes
  msh_bbox_t bbox_a = isect_get_transformed_aabbox( pc_a, pose_a );
  msh_bbox_t bbox_b = isect_get_transformed_aabbox( pc_b, pose_b );

  if( mshgeo_bbox_intersect(&bbox_a, &bbox_b) )
  { 
    // merge them
    msh_bbox_t bbox_c = mshgeo_bbox_init();
    mshgeo_bbox_union(&bbox_c, bbox_a.min_p);
    mshgeo_bbox_union(&bbox_c, bbox_a.max_p);
    mshgeo_bbox_union(&bbox_c, bbox_b.min_p);
    mshgeo_bbox_union(&bbox_c, bbox_b.max_p);

    isect_grid3d_t grid_a = {0};
    isect_grid3d_t grid_b = {0};
    isect_grid3d_t grid_c = {0};
    
    isect_grid3d_init( &grid_a, &bbox_c, voxel_size );
    isect_grid3d_init( &grid_b, &bbox_c, voxel_size );
    isect_grid3d_init( &grid_c, &bbox_c, voxel_size );

    int32_t grid_a_count, grid_b_count;
    if( voxelize_inside )
    {
      grid_a_count  = isect_compute_occupancy_grid( pc_a, pose_a, &grid_a );
      grid_b_count  = isect_compute_occupancy_grid( pc_b, pose_b, &grid_b );
    }
    else
    {
      grid_a_count  = isect_compute_boundary_grid( pc_a, pose_a, &grid_a );
      grid_b_count  = isect_compute_boundary_grid( pc_b, pose_b, &grid_b );
    }
    int32_t overlap_count = isect_compute_overlap_grid( &grid_a, &grid_b, &grid_c );
    int32_t denom = normalize_by_smaller ? msh_min(grid_a_count, grid_b_count)
                                         : msh_max(grid_a_count, grid_b_count);

    if (denom > 0) { overlap = (float)overlap_count / (float)denom; }
    else           { printf("WARNING: Grid A count: %d\n" 
                            "         Grid B count: %d\n"
                            "         Grid C count: %d\n", 
                            grid_a_count, grid_b_count, overlap_count); overlap = 1.0f; }

    isect_grid3d_term(&grid_a); 
    isect_grid3d_term(&grid_b); 
    isect_grid3d_term(&grid_c); 
  }
  else
  {
    overlap = 0.0f;
  }
  return overlap;
}

#endif /* INTERSECTION_IMPLEMENTATION */