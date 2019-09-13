#include <algorithm>

#include "stdio.h"
#include "stdint.h"
#include "math.h"

#include "mg/hashtable.h"
#include "msh/msh_std.h"
#include "msh/msh_vec_math.h"
#include "msh/msh_geometry.h"
#include "msh/msh_hash_grid.h"
#include "rs_pointcloud.h"
#include "rs_distance_function.h"
#include "rs_database.h"
#include "GCoptimization.h" 

#include "rs_pointcloud_filters.h"

#define RSPF_MAX_N_MODELS 64
#define RSPF_MAX_INSTANCES 1024
#define RSPF_POINTCLOUD_LEVEL 1

// disjoint-set forests using union-by-rank and path compression (sort of).
typedef struct {
  int32_t rank;
  int32_t p;
  int32_t size;
} uni_elt;

class universe {
 public:
  universe(int32_t elements) {
    elts = new uni_elt[elements];
    num = elements;
    for (int32_t i = 0; i < elements; i++) {
      elts[i].rank = 0;
      elts[i].size = 1;
      elts[i].p = i;
    }
  }
  ~universe() { delete [] elts; }
  int32_t find(int32_t x) {
    int32_t y = x;
    while (y != elts[y].p)
      y = elts[y].p;
    elts[x].p = y;
    return y;
  }
  void join(int32_t x, int32_t y) {
    if (elts[x].rank > elts[y].rank) {
      elts[y].p = x;
      elts[x].size += elts[y].size;
    } else {
      elts[x].p = y;
      elts[y].size += elts[x].size;
      if (elts[x].rank == elts[y].rank)
        elts[y].rank++;
    }
    num--;
  }
  int32_t size(int32_t x) const { return elts[x].size; }
  int32_t num_sets() const { return num; }
 private:
  uni_elt *elts;
  int32_t num;
};

bool operator<(const rspf_edge_t &a, const rspf_edge_t &b) {
  return a.weight < b.weight;
}

static int32_t 
rspf__edge_to_idx( rspf_edge_t* e, int32_t max_pts )
{
  int32_t idx1 = msh_max(e->idx1, e->idx2);
  int32_t idx2 = msh_min(e->idx1, e->idx2);
  return ( idx1 * max_pts + idx2 );
}

/* PLANES */

typedef struct rspf_detector_params
{
  msh_vec3_t* pts;
  msh_vec3_t* nrmls;
  size_t n_pts;
  float dot_threshold;
  float dist_threshold;
  size_t count_threshold;
  bool check_validity;
  bool check_extends;
} rspf_detector_params_t;



void
remove_inliers( rspf_plane_model_t* model, double* weights, 
                msh_vec3_t* pts, size_t n_pts, 
                float dist_threshold )
{
  msh_vec3_t p = model->plane.center;
  msh_vec3_t n = model->plane.normal;
  for( size_t i = 0; i < n_pts; ++i )
  {
    if( weights[i] > 0.01 )
    {
      float dist = msh_abs( msh_vec3_dot( n, msh_vec3_sub( pts[i], p ) ) );
      if( dist < dist_threshold )
      {
        weights[i] = 0.0f;
      }
    }
  }
}

void
evaluate_plane_model( rspf_plane_model_t* model, double* weights,
                      msh_vec3_t* pts, size_t n_pts, 
                      float dist_threshold )
{
  msh_vec3_t p = model->plane.center;
  msh_vec3_t n = model->plane.normal;
  for( size_t i = 0; i < n_pts; ++i )
  {
    if( weights[i] > 0.01 )
    {
      float dist = msh_abs( msh_vec3_dot( n, msh_vec3_sub( pts[i], p ) ) );
      if( dist < dist_threshold )
      {
        model->n_inliers++;
      }
    }
  }
}

int32_t
rspf__detect_walls( const rspf_detector_params_t* params, msh_array(rspf_plane_model_t)* models )
{
  double* weights = (double*)malloc( params->n_pts * sizeof(double) );
  msh_vec3_t up = msh_vec3_posy();
  for( size_t i = 0; i < params->n_pts; ++i )
  {
    float dot = msh_vec3_dot( params->nrmls[i], up );
    if( msh_abs(dot) < (1 - params->dot_threshold) ) { weights[i] = 1.0; }
    else                                             { weights[i] = 0.0; }
  }
  
  int32_t wall_count = 0;
  uint32_t max_ransac_iter = 5000;
  rspf_plane_model_t best_wall_model= {0};
  do
  {
    msh_discrete_distrib_t dist = {0};
    msh_discrete_distribution_init( &dist, weights, params->n_pts, 12346ULL );

    rspf_plane_model_t wall_model = {0};
    best_wall_model.n_inliers = 0;
    int32_t wall_detected = 0;
    for( uint32_t i = 0; i < max_ransac_iter; ++i )
    {
      int32_t idx_a, idx_b, idx_c;
      idx_a = msh_discrete_distribution_sample( &dist );
      do { idx_b = msh_discrete_distribution_sample( &dist ); } while( idx_a == idx_b );
      do { idx_c = msh_discrete_distribution_sample( &dist ); } while( idx_b == idx_c );

      msh_vec3_t p_a = params->pts[idx_a];
      msh_vec3_t p_b = params->pts[idx_b];
      msh_vec3_t p_c = params->pts[idx_c];

      msh_vec3_t v_a = msh_vec3_sub( p_b, p_a );
      msh_vec3_t v_b = msh_vec3_sub( p_c, p_a );
      msh_vec3_t n = msh_vec3_normalize( msh_vec3_cross( v_a, v_b ) );

      wall_model.plane.center = p_a;
      wall_model.plane.normal = n;
      wall_model.n_inliers = 0;
      if( msh_abs( msh_vec3_dot( n, up ) ) < (1 - params->dot_threshold) )
      {
        evaluate_plane_model( &wall_model, weights, 
                              params->pts, params->n_pts, params->dist_threshold );
        if( wall_model.n_inliers > best_wall_model.n_inliers )
        {
          best_wall_model = wall_model;
          wall_detected = 1;
        }
      }
    }

    msh_discrete_distribution_free( &dist );
    if( wall_detected ) { msh_array_push( (*models), best_wall_model ); }
    
    remove_inliers( &best_wall_model, weights, 
                    params->pts, params->n_pts, params->dist_threshold );
    wall_count++;

  } while( best_wall_model.n_inliers > params->count_threshold );
  msh_array_pop( (*models) );
  wall_count--;

  free( weights );
  return wall_count;
}

int32_t
rspf__detect_floor( const rspf_detector_params_t* params, msh_array(rspf_plane_model_t)* models )
{
  double* weights = (double*)malloc( params->n_pts * sizeof(double) );
  msh_vec3_t up = msh_vec3_posy();
  for( size_t i = 0; i < params->n_pts; ++i )
  {
    float dot = msh_vec3_dot( params->nrmls[i], up );
    if( dot > params->dot_threshold ) { weights[i] = 1.0; }
    else                              { weights[i] = 0.0; }
  }

  msh_discrete_distrib_t dist = {0};
  msh_discrete_distribution_init( &dist, weights, params->n_pts, 12346ULL );

  uint32_t max_ransac_iter = 2500;
  rspf_plane_model_t best_floor_model= {0};
  rspf_plane_model_t floor_model= {0};
  int32_t floor_count = 0;
  for( uint32_t i = 0; i < max_ransac_iter; ++i )
  {
    int32_t idx_a = msh_discrete_distribution_sample( &dist );
    int32_t idx_b = msh_discrete_distribution_sample( &dist );
    int32_t idx_c = msh_discrete_distribution_sample( &dist );

    msh_vec3_t p_a = params->pts[idx_a];
    msh_vec3_t p_b = params->pts[idx_b];
    msh_vec3_t p_c = params->pts[idx_c];

    msh_vec3_t v_a = msh_vec3_sub( p_b, p_a );
    msh_vec3_t v_b = msh_vec3_sub( p_c, p_a );
    msh_vec3_t n = msh_vec3_normalize( msh_vec3_cross( v_a, v_b ) );

    floor_model.plane.center = p_a;
    floor_model.plane.normal = n;
    floor_model.n_inliers = 0;
    evaluate_plane_model( &floor_model, weights, params->pts, params->n_pts, params->dist_threshold );
    if( floor_model.n_inliers > best_floor_model.n_inliers )
    {
      best_floor_model = floor_model;
      floor_count = 1;
    }
  }

  free( weights );
  msh_discrete_distribution_free( &dist );
  if( floor_count) { msh_array_push( (*models), best_floor_model ); }
  return floor_count;
}

bool
rspf__is_point_within_convex_poly( msh_vec3_t p,  msh_vec3_t* poly, size_t n_verts )
{
  assert( n_verts > 2 );
  for( size_t i = 0; i < n_verts - 1; ++i )
  {
    msh_vec3_t a = poly[i];
    msh_vec3_t b = poly[i+1];
    msh_vec3_t c = poly[(i+2)%n_verts];
    msh_vec3_t v1 = msh_vec3_sub( b, a );
    msh_vec3_t v2 = msh_vec3_sub( c, b );
    msh_vec3_t v3 = msh_vec3_sub( p, b );
    msh_vec3_t n1 = msh_vec3_cross( v1, v2 );
    msh_vec3_t n2 = msh_vec3_cross( v1, v3 );
    float val = msh_vec3_dot(n1, n2);
    if( val < 0 )
    {
      return false;
    }
  }
  return true;
}

void
rspf__gather_model_inliers( const rspf_detector_params_t* params,
                            msh_array(rspf_plane_model_t)* models )
{
  for( size_t mi = 0 ; mi < msh_array_len((*models)) ; mi++ )
  {
    rspf_plane_model_t* model = (*models) + mi;
    if( params->check_validity && !model->valid ) { continue; }
    msh_vec3_t poly[4];
    if( params->check_extends )
    {
      msh_vec3_t o = model->plane.center;
      msh_vec3_t pos_x = msh_vec3_scalar_mul( model->axes.col[0], model->extends.x );
      msh_vec3_t pos_y = msh_vec3_scalar_mul( model->axes.col[1], model->extends.y );
      msh_vec3_t neg_x = msh_vec3_scalar_mul( model->axes.col[0], model->extends.z );
      msh_vec3_t neg_y = msh_vec3_scalar_mul( model->axes.col[1], model->extends.w );
      poly[0] = msh_vec3_add( msh_vec3_add( o, pos_x ), pos_y );
      poly[1] = msh_vec3_add( msh_vec3_add( o, pos_x ), neg_y );
      poly[2] = msh_vec3_add( msh_vec3_add( o, neg_x ), neg_y );
      poly[3] = msh_vec3_add( msh_vec3_add( o, neg_x ), pos_y );
    }
    for( size_t i = 0; i < params->n_pts; ++i )
    {
      msh_vec3_t offset = msh_vec3_sub( params->pts[i], model->plane.center );
      float dist = msh_abs( msh_vec3_dot( model->plane.normal, offset ) );
      float dot = msh_abs( msh_vec3_dot( params->nrmls[i], model->plane.normal ));
      if( dot  > params->dot_threshold && 
          dist < params->dist_threshold )
      {
        if( params->check_extends )
        {
          msh_vec3_t p0 = params->pts[i];
          bool is_within = rspf__is_point_within_convex_poly( p0, poly, 4 );
          if( is_within )
          {
            msh_array_push( model->inlier_ind, i );
          }
        }
        else
        {
          msh_array_push( model->inlier_ind, i );
        }
      }
    }
    model->n_inliers = msh_array_len( model->inlier_ind );
  }
}

void
rspf__split_by_connected_components( const rspf_detector_params_t* params, 
                                     msh_array(rspf_plane_model_t)* models )
{
  msh_array( rspf_plane_model_t ) new_models = {0};
  int32_t max_nn = 8;
  for( size_t mi = 0 ; mi < msh_array_len((*models)); mi++ )
  {
    // gather current points
    msh_array(msh_vec3_t) cur_pts = {0};
    rspf_plane_model_t* model = (*models) + mi;
    for( size_t i = 0; i < model->n_inliers ; ++i )
    {
      msh_array_push( cur_pts, params->pts[model->inlier_ind[i]] );
    }

    // create edges between local neighbors
    float radius = 0.05f;
    msh_hash_grid_t search_grid = {0};
    msh_hash_grid_init_3d( &search_grid, (float*)cur_pts, model->n_inliers, radius );

    hashtable_t edge_table;
    hashtable_init( &edge_table, sizeof(rspf_edge_t), 1024, NULL);

    float* nn_dist = (float*)malloc( model->n_inliers * max_nn * sizeof(float) );
    int32_t* nn_inds = (int32_t*)malloc( model->n_inliers * max_nn * sizeof(int32_t) );
    size_t* n_neighbors = (size_t*)malloc( model->n_inliers * sizeof(size_t));

    msh_hash_grid_search_desc_t search_opts = {0};
    search_opts.query_pts = &cur_pts[0].x;
    search_opts.n_query_pts =model->n_inliers;
    search_opts.distances_sq = nn_dist;
    search_opts.indices = nn_inds;
    search_opts.n_neighbors = n_neighbors;
    search_opts.max_n_neigh = max_nn;
    search_opts.radius = radius;
    msh_hash_grid_radius_search( &search_grid, &search_opts );

    for( size_t i = 0 ; i < model->n_inliers ; ++i )
    {
      for( size_t j = 1 ; j < n_neighbors[i] ; ++j )
      {
        size_t idx = i * max_nn + j;
        if( nn_inds[idx] < 0 ) { break; }

        rspf_edge_t e = { (int32_t)i, nn_inds[idx] };
        int32_t eidx = rspf__edge_to_idx( &e, model->n_inliers );
        rspf_edge_t* e_old = (rspf_edge_t*)hashtable_find( &edge_table, (HASHTABLE_U64)eidx );
        if(!e_old) hashtable_insert(&edge_table, eidx, &e );
      }
    }
    free( nn_dist );
    free( nn_inds );
    free( n_neighbors );
    msh_hash_grid_term( &search_grid );

    size_t n_edges = hashtable_count(&edge_table);
    rspf_edge_t* edges = (rspf_edge_t*)hashtable_items( &edge_table );

    // Gather connected components
    universe *u = new universe( model->n_inliers );  // make a disjoint-set forest
    for( size_t i = 0; i < n_edges; i++ )
    {
      rspf_edge_t *pedge = &edges[i];
      // components connected by this edge
      int32_t a = u->find( pedge->idx1 );
      int32_t b = u->find( pedge->idx2 );
      if (a != b) { u->join(a, b); }
    }

    hashtable_t unique_components_table;
    hashtable_init( &unique_components_table, sizeof(rspf_plane_model_t), 1024, NULL);
    for( size_t i = 0; i < model->n_inliers; ++i )
    {
      size_t component_id = u->find(i);
      rspf_plane_model_t* model_ptr = (rspf_plane_model_t*)hashtable_find( &unique_components_table, component_id );
      if( !model_ptr ) 
      { 
        rspf_plane_model_t new_model = {0};
        new_model.plane = model->plane;
        new_model.n_inliers = u->size(component_id);
        new_model.inlier_ind = {0};
        msh_array_push( new_model.inlier_ind, model->inlier_ind[i] );
        hashtable_insert( &unique_components_table, component_id, &new_model );
      }
      else
      {
        msh_array_push( model_ptr->inlier_ind, model->inlier_ind[i] );
      }
    }

    rspf_plane_model_t* unique_components = (rspf_plane_model_t*)hashtable_items( &unique_components_table );
    for( int32_t i = 0; i < hashtable_count( &unique_components_table ); ++i )
    {
      msh_array_push( new_models, unique_components[i] );
    }

    
    hashtable_term( &unique_components_table );
    hashtable_term( &edge_table );
    delete u;
    msh_array_free( cur_pts );
  }

  // Free up the old models and reassing
  for( size_t i = 0; i < msh_array_len((*models)); ++i )
  {
    msh_array_free( (*models)->inlier_ind );
  }
  msh_array_free( (*models) );
  (*models) = new_models;
}

void
rspf__refine_plane_models( const rspf_detector_params_t* params, msh_array(rspf_plane_model_t)* models )
{
  for( size_t mi = 0; mi < msh_array_len((*models)); ++mi )
  {
    rspf_plane_model_t* model = (*models) + mi;
    model->n_inliers = msh_array_len( model->inlier_ind );
    
    // average normal + average center
    msh_vec3_t new_center = msh_vec3_zeros();
    msh_vec3_t new_normal = msh_vec3_zeros();
    for( size_t i = 0; i < model->n_inliers; ++i )
    {
      new_center = msh_vec3_add( params->pts[model->inlier_ind[i]], new_center );
      new_normal = msh_vec3_add( params->nrmls[model->inlier_ind[i]], new_normal);
    }
    model->plane.center = msh_vec3_scalar_div( new_center, model->n_inliers );
    model->plane.normal = msh_vec3_normalize(msh_vec3_scalar_div( new_normal, model->n_inliers ) );
    
    // Calculate axes
    model->axes.col[2] = model->plane.normal;
    if( msh_vec3_dot( model->plane.normal, msh_vec3_posy() ) > 0.8 ) 
    {
      model->axes.col[1] = msh_vec3_posz();
      model->axes.col[0] = msh_vec3_normalize( msh_vec3_cross( model->axes.col[1], model->axes.col[2] ) );
      model->axes.col[1] = msh_vec3_normalize( msh_vec3_cross( model->axes.col[0], model->axes.col[2] ) );
    }
    else
    {
      model->axes.col[1] = msh_vec3_posy();
      model->axes.col[0] = msh_vec3_normalize( msh_vec3_cross( model->axes.col[1], model->axes.col[2] ) );
      model->axes.col[1] = msh_vec3_normalize( msh_vec3_cross( model->axes.col[0], model->axes.col[2] ) );
    }

    // calculate extends
    msh_mat3_t R = msh_mat3_transpose( model->axes );
    msh_vec3_t t = msh_vec3_invert( model->plane.center );
    float max_x, max_y, min_x, min_y;
    max_x = max_y = -MSH_F32_MAX;
    min_x = min_y = MSH_F32_MAX;
    for( size_t i = 0; i < model->n_inliers; ++i )
    {
      msh_vec3_t pos = params->pts[model->inlier_ind[i]];
      pos = msh_vec3_add( pos, t );
      pos = msh_mat3_vec3_mul( R, pos );
      max_x = msh_max( pos.x, max_x );
      max_y = msh_max( pos.y, max_y );
      min_x = msh_min( pos.x, min_x );
      min_y = msh_min( pos.y, min_y );
    }
    model->extends = msh_vec4( max_x, max_y, min_x, min_y );
  }
}

void
rspf_detect_planes( const rs_pointcloud_t* pc, 
                    msh_array( rspf_plane_model_t) *plane_models )
{
  uint64_t t1, t2;
  t1 = msh_time_now();
  rspf_detector_params_t params = {0};
  int32_t lvl              = 2;
  params.pts               = pc->positions[lvl];
  params.nrmls             = pc->normals[lvl];
  params.n_pts             = pc->n_pts[lvl];
  params.dot_threshold     = 0.8;
  params.dist_threshold    = 0.033;
  params.count_threshold   = 250;
  int32_t n_floors = rspf__detect_floor( &params, plane_models );
  int32_t n_walls  = rspf__detect_walls( &params, plane_models );
  t2 = msh_time_now();
  printf( "RSPF_PLANE_DETECTOR: Found %d large planes in %f sec.\n", n_floors + n_walls, msh_time_diff_sec(t2, t1) );

  t1 = msh_time_now();
  lvl                      = 0;
  params.pts               = pc->positions[lvl];
  params.nrmls             = pc->normals[lvl];
  params.n_pts             = pc->n_pts[lvl];
  params.dist_threshold    = 0.05;
  rspf__gather_model_inliers( &params, plane_models );
  rspf__split_by_connected_components( &params, plane_models );
  t2 = msh_time_now();
  printf( "RSPF_PLANE_DETECTOR: Split %d large planes into %d connected components in %f sec.\n", 
          n_floors + n_walls, (int32_t)msh_array_len((*plane_models)), msh_time_diff_sec(t2, t1) );

  t1 = msh_time_now();
  rspf__refine_plane_models( &params, plane_models );
  t2 = msh_time_now();
  printf("RSPF_PLANE_DETECTOR: Refining planes took %f sec.\n", msh_time_diff_sec(t2, t1) );

}

void
rspf_compute_plane_features( const rs_pointcloud_t* pc, 
                             msh_array( rspf_plane_model_t) *planes )
{
  for( size_t i = 0; i < msh_array_len((*planes)); ++i )
  {
    rspf_plane_model_t* model = (*planes) + i;
    
    msh_vec3_t up = msh_vec3_posy();
    model->features.normal_up_dot = msh_vec3_dot( model->plane.normal, up );

    model->features.count = model->n_inliers;

    msh_vec3_t o = model->plane.center;
    msh_vec3_t pos_x = msh_vec3_scalar_mul( model->axes.col[0], model->extends.x );
    msh_vec3_t pos_y = msh_vec3_scalar_mul( model->axes.col[1], model->extends.y );
    msh_vec3_t neg_x = msh_vec3_scalar_mul( model->axes.col[0], model->extends.z );
    msh_vec3_t neg_y = msh_vec3_scalar_mul( model->axes.col[1], model->extends.w );
    msh_vec3_t a = msh_vec3_add( msh_vec3_add( o, pos_x ), pos_y );
    msh_vec3_t b = msh_vec3_add( msh_vec3_add( o, pos_x ), neg_y );
    msh_vec3_t c = msh_vec3_add( msh_vec3_add( o, neg_x ), pos_y );
    msh_vec3_t d = msh_vec3_add( msh_vec3_add( o, neg_x ), neg_y );
    float max_y = msh_max( a.y, b.y );
    max_y = msh_max( max_y, c.y );
    max_y = msh_max( max_y, d.y );
    model->features.max_y = max_y;

    int32_t lvl = 0;
    size_t saliency_count = 0;
    for( size_t i = 0; i < model->features.count; ++i )
    {
      saliency_count += pc->qualities[lvl][model->inlier_ind[i]];
    }
    model->features.saliency = (double)saliency_count / (double)model->features.count;
  }
}


void
rspf_classify_planes( const rs_pointcloud_t* pc, 
                      msh_array( rspf_plane_model_t) *planes )
{
  int32_t lvl = 0;
  size_t inlier_threshold  = 2000;
  float dot_threshold      = 0.7;
  float height_threshold   = 0.9;
  float saliency_threshold = 0.9;

  for( size_t i = 0; i < msh_array_len((*planes)); ++i )
  {
    rspf_plane_model_t* model = (*planes) + i;
    rspf_plane_feature_set_t* features = &model->features;
    model->valid = 0;

    int32_t wall_class  = 1;
    int32_t floor_class = 2;
    int32_t cur_class   = 0;
    if( features->count >= inlier_threshold ) 
    {
      if( features->normal_up_dot > dot_threshold )
      {
        cur_class = floor_class;
      }
      else
      {
        if( features->saliency < saliency_threshold && 
            features->max_y >= height_threshold) 
        { 
          cur_class = wall_class;
        }
      }
    }

    if( cur_class != 0 )
    {
      model->valid = 1;
      for( size_t pi = 0; pi < features->count; ++pi )
      {
        pc->class_ids[lvl][ model->inlier_ind[pi] ] = cur_class;
        pc->instance_ids[lvl][ model->inlier_ind[pi] ] = i;
        pc->qualities[lvl][ model->inlier_ind[pi] ] = 0.0f;
      }
    }
  }
}


void
rspf_relabel_walls_and_floors( const rsdb_t* rsdb,
                               const rs_pointcloud_t* pc,
                               msh_array( rspf_plane_model_t) *planes )
{
  uint64_t t1, t2;
  float dot_threshold = 0.8;
  int32_t floor_idx      = rsdb_get_class_idx( rsdb, "floor" );
  int32_t wall_idx       = rsdb_get_class_idx( rsdb, "wall" );
  int32_t unlabelled_idx = rsdb_get_class_idx( rsdb, "unlabelled" );
  t1 = msh_time_now();
  for( size_t pi = 0; pi < msh_array_len((*planes)); ++pi )
  {
    msh_array_free( (*planes)[pi].inlier_ind );
  }
  rspf_detector_params_t params = {0};
  int32_t lvl              = 1;
  params.pts               = pc->positions[lvl];
  params.nrmls             = pc->normals[lvl];
  params.n_pts             = pc->n_pts[lvl];
  params.dist_threshold    = 0.05;
  params.check_validity    = 1;
  params.check_extends     = 1;
  rspf__gather_model_inliers( &params, planes );
  t2 = msh_time_now();
  printf( "RSPF_WALL_FLOOR_RELABEL: Gathering model inlierson level %d took %f sec.\n", 
           lvl, msh_time_diff_sec(t2, t1) );

  t1 = msh_time_now();
  for( size_t mi = 0; mi < msh_array_len((*planes)); ++mi )
  {
    rspf_plane_model_t* model = (*planes) + mi;
    if( model->valid )
    {
      for( size_t i = 0; i < msh_array_len(model->inlier_ind); ++i )
      {
        int32_t* instance_idx = &pc->instance_ids[lvl][model->inlier_ind[i]];
        int32_t* class_idx = &pc->class_ids[lvl][model->inlier_ind[i]];
        if( model->features.normal_up_dot > dot_threshold )
        {
          if( *instance_idx >= 1024 ) *instance_idx = 0;
          if( *class_idx == unlabelled_idx ) *class_idx = floor_idx;
        }
        else
        {
          if( *instance_idx >= 1024 ) *instance_idx = 1;
          if( *class_idx == unlabelled_idx ) *class_idx = wall_idx;
        }
      }
    }
  }
  t2 = msh_time_now();
  printf( "RSPF_WALL_FLOOR_RELABEL: Relabeling walls and floors took %f sec.\n", 
          msh_time_diff_sec(t2, t1) );
}

/* LABELS */
msh_array(rspf_edge_t)
rspf_compute_neighborhood( const rs_pointcloud_t* pc, int32_t lvl, int32_t max_nn, 
                          float radius_sq, float dist_exp, float angle_exp )
{
  hashtable_t edge_table;
  hashtable_init(&edge_table, sizeof(rspf_edge_t), 1024, NULL);

  float* nn_dist = (float*)malloc( pc->n_pts[lvl]*max_nn*sizeof(float) );
  int32_t* nn_inds = (int32_t*)malloc( pc->n_pts[lvl]*max_nn*sizeof(int32_t) );
  size_t* n_neighbors = (size_t*)malloc( pc->n_pts[lvl]*sizeof(size_t));

  msh_hash_grid_search_desc_t search_opts = {0};
  search_opts.query_pts  = &pc->positions[lvl][0].x;
  search_opts.n_query_pts = pc->n_pts[lvl];
  search_opts.distances_sq = nn_dist;
  search_opts.indices = nn_inds;
  search_opts.n_neighbors = n_neighbors;
  search_opts.max_n_neigh = max_nn;
  search_opts.radius = sqrt(radius_sq);
  msh_hash_grid_radius_search( pc->search_grids[lvl], &search_opts );


  for( size_t i = 0 ; i < pc->n_pts[lvl] ; ++i )
  {
    msh_vec3_t n = pc->normals[lvl][i];
    
    for(size_t j = 0 ; j < n_neighbors[i] ; ++j)
    {
      size_t idx = i * max_nn + j;
      if( nn_inds[idx] < 0 || nn_inds[idx] > (int32_t)pc->n_pts[lvl] ) break;
      
      msh_vec3_t m  = pc->normals[lvl][nn_inds[idx]];
      float dist_cost  = 1.0f - pow(nn_dist[idx]/(4.0*radius_sq), dist_exp);
      float norm_cost  = pow( msh_clamp(msh_vec3_dot(n,m), 0.0f, 1.0f), angle_exp);
      float cost = dist_cost * norm_cost;
      rspf_edge_t e = { (int32_t)i, nn_inds[idx], cost};
      int32_t eidx = rspf__edge_to_idx(&e, pc->n_pts[lvl]);
      rspf_edge_t* e_old = (rspf_edge_t*)hashtable_find(&edge_table, (HASHTABLE_U64)eidx);
      if(!e_old) hashtable_insert(&edge_table, eidx, &e );
    }
  }
  size_t n_edges = hashtable_count(&edge_table);
  
  msh_array(rspf_edge_t) edges = NULL;
  msh_array_copy(edges, hashtable_items(&edge_table), n_edges );
  hashtable_term(&edge_table);

  return edges;
}

static rsdb_t* RSFP_SORT_PTR = NULL;
int32_t rsfp__static_plcmnt_cmp( const void* a, const void* b )
{
  rs_obj_plcmnt_t* plcmnt_a = (rs_obj_plcmnt_t*)a;
  rs_obj_plcmnt_t* plcmnt_b = (rs_obj_plcmnt_t*)b;
  int32_t is_static_a = rsdb_is_object_static( RSFP_SORT_PTR, plcmnt_a->object_idx);
  int32_t is_static_b = rsdb_is_object_static( RSFP_SORT_PTR, plcmnt_b->object_idx);
  int32_t class_idx_a = RSFP_SORT_PTR->objects[ plcmnt_a->object_idx ].class_idx;
  int32_t class_idx_b = RSFP_SORT_PTR->objects[ plcmnt_b->object_idx ].class_idx;
  int32_t cmp_a = is_static_b << 10 | class_idx_b;
  int32_t cmp_b = is_static_a << 10 | class_idx_a;
  return cmp_b - cmp_a;
}

void
rspf__assign_temporary_labels( rsdb_t* rsdb, rs_pointcloud_t* pc,
                               msh_array(rs_obj_plcmnt_t) arrangement,
                               msh_hash_grid_search_desc_t* search_opts,
                               int8_t* labels, float* min_dists,
                               size_t start, size_t end, int32_t lvl )
{
  for( size_t i = start; i < end; ++i )
  {
    int32_t object_idx = arrangement[i].object_idx;
    rs_pointcloud_t* shape = rsdb->objects[object_idx].shape;
    msh_mat4_t xform = arrangement[i].pose;
    msh_mat4_t inv_xform = msh_mat4_inverse(xform);
    msh_mat4_t normal_matrix = msh_mat4_transpose( xform );
    msh_vec3_t* pts = (msh_vec3_t*)search_opts->query_pts;
    for( size_t j = 0; j < search_opts->n_query_pts; ++j ) 
    { 
      pts[j] = msh_mat4_vec3_mul( inv_xform, pc->positions[lvl][j], 1 ); 
    }

    msh_hash_grid_radius_search( shape->search_grids[lvl], search_opts );

    for( size_t j = 0; j < search_opts->n_query_pts; ++j )
    {
      if( search_opts->n_neighbors[j] > 0 && 
          search_opts->distances_sq[j] < min_dists[j] )
      {
        msh_vec3_t n1 = pc->normals[lvl][j];
        n1 = msh_mat4_vec3_mul( normal_matrix, n1, 0 );
        msh_vec3_t n2 = shape->normals[lvl][search_opts->indices[j]];
        //TODO(maciej): I don't think we should be forced to nromalize them... Is there something wrong with the inverse xform?
        float angle = acos( fabs( msh_vec3_dot( msh_vec3_normalize(n1), msh_vec3_normalize(n2) ) ) );
        if( angle < msh_deg2rad(70.0) )
        {
          min_dists[j] = search_opts->distances_sq[j];
          labels[j] = i+1;
        }
      }
    }
  }
}

void
rspf_arrangement_to_labels( rsdb_t* rsdb, 
                            rs_pointcloud_t* in_pc, 
                            msh_array(rs_obj_plcmnt_t) arrangement, 
                            float radius /* = 0.025*/,
                            bool prioritize_static )
{
  printf("LABEL_TRANSFER:   Starting copying labels from arrangement...\n");

  int32_t lvl = RSPF_POINTCLOUD_LEVEL;
  int32_t n_pts = in_pc->n_pts[lvl];
  size_t n_placements = msh_array_len(arrangement);
  
  uint64_t t1, t2;
  t1 = msh_time_now();
  
  // select labels by nearest neighbor lookup
  int8_t* labels     = (int8_t*)malloc( n_pts*sizeof(int8_t) );

  for( int32_t i = 0 ; i < n_pts; ++i)
  {
    labels[i] = 0;
  }

  int32_t max_n_neigh = 1;
  msh_vec3_t* pts = (msh_vec3_t*)malloc( n_pts * sizeof(msh_vec3_t) );
  int32_t* indices = (int32_t*)malloc( max_n_neigh * n_pts * sizeof(int32_t) );
  float* dists_sq = (float*)malloc( max_n_neigh * n_pts * sizeof(float) );
  size_t* n_neighbors = (size_t*)malloc( n_pts * sizeof(size_t));
  float* min_dists = (float*)malloc( n_pts * sizeof(float) );
  
  msh_hash_grid_search_desc_t search_opts = {0};
  search_opts.query_pts    = &pts[0].x;
  search_opts.n_query_pts  = n_pts;
  search_opts.distances_sq = dists_sq;
  search_opts.indices      = indices;
  search_opts.n_neighbors  = n_neighbors;
  search_opts.max_n_neigh  = max_n_neigh;
  search_opts.radius       = radius;

  for( int32_t i = 0; i < n_pts; ++i ) { min_dists[i] = 1e9; }

  // Sort copied array
  msh_array( rs_obj_plcmnt_t ) arrangement_cpy = {0};
  msh_array_copy( arrangement_cpy, arrangement, n_placements );
  RSFP_SORT_PTR = rsdb;
  qsort( arrangement_cpy, n_placements, sizeof((arrangement_cpy)[0]), rsfp__static_plcmnt_cmp );
  RSFP_SORT_PTR = NULL;

  // Find index of the first static object in the arrangement
  int32_t first_static_obj_idx = 0;
  for( size_t i = 0; i < n_placements; ++i )
  {
    int32_t obj_idx = arrangement_cpy[i].object_idx;
    if( rsdb_is_object_static( rsdb, obj_idx ) ) { first_static_obj_idx = i; break; }
  }

  rspf__assign_temporary_labels( rsdb, in_pc, arrangement_cpy, &search_opts,
                                 labels, min_dists, 
                                 0, first_static_obj_idx, lvl );

  if( prioritize_static )
  {
    for( int32_t i = 0; i < n_pts; ++i ) { min_dists[i] = 1e9; }
  }
  if( !prioritize_static ) { search_opts.radius = 1.5f * radius; }
  rspf__assign_temporary_labels( rsdb, in_pc, arrangement_cpy, &search_opts,
                                 labels, min_dists, 
                                 first_static_obj_idx, n_placements, lvl );

  // temporary labels to class/instance ids
  for( int32_t i = 0 ; i < n_pts; ++i)
  {
    int32_t class_idx     = -1;
    int32_t instance_idx  = -1;
    if( labels[i] == 0 ) 
    { 
      class_idx = rsdb_get_class_idx(rsdb, "unlabelled"); 
      instance_idx = RSPF_MAX_INSTANCES; 
    }
    else {
      int32_t label_idx = labels[i] - 1;
      rs_obj_plcmnt_t* obj_plcmnt = &arrangement_cpy[label_idx];
      rs_object_t* object = &rsdb->objects[obj_plcmnt->object_idx];
      class_idx     = object->class_idx;
      instance_idx  = obj_plcmnt->uidx;
    }
    in_pc->class_ids[lvl][i]    = class_idx;
    in_pc->instance_ids[lvl][i] = instance_idx;
  }
  t2 = msh_time_now();

  free( min_dists );
  free( pts );
  free( indices );
  free( dists_sq );
  free( labels );
  msh_array_free( arrangement_cpy );
  printf( "LABEL_TRANSFER:   Done in %fms\n", msh_time_diff_ms( t2, t1 ) );
}


void 
rspf_smooth_labels( rsdb_t* rsdb, rs_pointcloud_t* in_pc )
{
  printf("LABEL_TRANSFER:   Performing label smoothing...\n");
  int32_t lvl = RSPF_POINTCLOUD_LEVEL;
  int32_t n_pts = in_pc->n_pts[lvl];
  uint64_t gt1, gt2, t1, t2;

  gt1 = msh_time_now();
  t1 = msh_time_now();

  // select labels by nearest neighbor lookup
  float radius = 0.05f;
  float radius_sq = radius * radius;
  int32_t* labels     = (int32_t*)malloc(n_pts*sizeof(int32_t));
  int32_t max_uidx = -1;
  for( int32_t i = 0 ; i < n_pts; ++i)
  {
    if( in_pc->instance_ids[lvl][i] < RSPF_MAX_INSTANCES )
    { 
      max_uidx = msh_max( max_uidx, in_pc->instance_ids[lvl][i] ); 
    }
  }
  int32_t n_labels = max_uidx + 5;
  int32_t* label_to_class_map  = (int32_t*)malloc(n_labels * sizeof(int32_t));
  int32_t* label_to_instance_map  = (int32_t*)malloc(n_labels * sizeof(int32_t));
  for( int32_t i = 0 ; i < n_pts; ++i)
  {
    int32_t instance_idx = in_pc->instance_ids[lvl][i];
    int32_t class_idx = in_pc->class_ids[lvl][i];
    int32_t label_idx = instance_idx + 1;
    if( class_idx == rsdb_get_class_idx( rsdb, (const char*)"unlabelled") ) { label_idx = 0; }
    labels[i] = label_idx;
    label_to_class_map[labels[i]] = class_idx;
    label_to_instance_map[labels[i]] = instance_idx;
  }


  t1 = msh_time_now();
  msh_array(rspf_edge_t) edges = rspf_compute_neighborhood (in_pc, lvl, 8, radius_sq, 15.0f, 16.0f );
  t2 = msh_time_now();
  printf("LABEL_TRANSFER:      Neighborhood compatibility computation took %fms\n", msh_time_diff_ms(t2,t1));
  
  t1 = msh_time_now();
  // Setup unary term
  int32_t* data_cost = (int32_t*)malloc(n_pts*n_labels*sizeof(int32_t));
  for( int32_t i = 0; i < n_pts; i++ )
  {
    int32_t desired_label = labels[i];
    int32_t cost = 30;
    if( rsdb_is_class_static( rsdb, label_to_class_map[labels[i]] ) ) { cost = 15; }
    if( labels[i] == 0 ) { cost = 1; }
    // test this with previous dataset
    for( int32_t l = 0; l < n_labels; l++ )
    {
      data_cost[i*n_labels+l] = (l == desired_label) ? 0 : cost;
    }
  }

  // Setup pairwise term
  int32_t edge_cost = 10;
	int32_t *smooth_cost = (int32_t*)malloc(n_labels*n_labels*sizeof(int32_t));
  for ( int32_t l1 = 0; l1 < n_labels; l1++ )
  {
    for (int32_t l2 = 0; l2 < n_labels; l2++ )
    {
      smooth_cost[l1+l2*n_labels] = (l1 == l2) ? 0 : edge_cost;
    }
  }
  t2 = msh_time_now();
  printf("LABEL_TRANSFER:      Data and smoothness terms setting took %fms\n", msh_time_diff_ms(t2,t1));

  t1 = msh_time_now();
  GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph( n_pts, n_labels );
  gc->setDataCost(data_cost);
  gc->setSmoothCost(smooth_cost);
  
  // set initial labels
  for ( int32_t  i = 0; i < n_pts; i++ ) { gc->setLabel(i, labels[i]); }
  // upload pairwise cost
  for( size_t i = 0; i < msh_array_len(edges); ++i)
  {
    rspf_edge_t e = edges[i];
    gc->setNeighbors(e.idx1, e.idx2, (int32_t)(e.weight * edge_cost) );
  }
  printf("LABEL_TRANSFER:      Optimizing over %d pts. and %d labels\n", n_pts, n_labels );
  gc->swap(2);
  
  for ( int32_t i = 0; i < n_pts; i++ ) { labels[i] = gc->whatLabel(i); }
  delete gc;
  for( int32_t i = 0 ; i < n_pts; ++i)
  {
    int32_t class_idx     = label_to_class_map[labels[i]];
    int32_t instance_idx  = label_to_instance_map[labels[i]];
    in_pc->class_ids[lvl][i]    = class_idx;
    in_pc->instance_ids[lvl][i] = instance_idx;
  }
  t2 = msh_time_now();
  printf("LABEL_TRANSFER:      Label optimization took: %fs.\n", msh_time_diff_sec( t2, t1 ));

  gt2 = msh_time_now();
  printf("LABEL_TRANSFER:   Label smoothing took: %fs.\n", msh_time_diff_sec( gt2, gt1) );

  free(labels);
  msh_array_free(edges);
  free(data_cost);
  free(smooth_cost);
}

universe*
rspf__segment_graph(int32_t num_vertices, int32_t num_edges, rspf_edge_t *edges, float c) 
{
  std::sort(edges, edges + num_edges);  // sort edges by weight
  universe *u = new universe(num_vertices);  // make a disjoint-set forest
  float *threshold = new float[num_vertices];
  for (int32_t i = 0; i < num_vertices; i++) { threshold[i] = c; }
  // for each edge, in non-decreasing weight order
  for (int32_t i = 0; i < num_edges; i++) {
    rspf_edge_t *pedge = &edges[i];
    // components conected by this edge
    int32_t idx1 = u->find(pedge->idx1);
    int32_t idx2 = u->find(pedge->idx2);
    if (idx1 != idx2) {
      if ((pedge->weight <= threshold[idx1]) && (pedge->weight <= threshold[idx2])) {
        u->join(idx1, idx2);
        idx1 = u->find(idx1);
        threshold[idx1] = pedge->weight + (c / u->size(idx1));
      }
    }
  }
  delete [] threshold;
  return u;
}
