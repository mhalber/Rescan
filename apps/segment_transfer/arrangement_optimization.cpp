#define INTERSECTION_IMPLEMENTATION
#define MSH_GEOMETRY_IMPLEMENTATION

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cassert>

#include "msh/msh_std.h"
#include "msh/msh_vec_math.h"
#include "msh/msh_geometry.h"
#include "msh/msh_hash_grid.h"
#include "mg/hashtable.h"
#include "rs_pointcloud.h"
#include "rs_distance_function.h"
#include "rs_database.h"
#include "intersect.h"

#include "arrangement_optimization.h"

#define RSAO_EPS 0.000001

enum rsao__object_type
{
  RSAO_DYNAMIC_OBJECT = 0,
  RSAO_STATIC_OBJECT = 1
};

enum rsao__cell_value
{
  RSAO_CELL_INACTIVE = 0,
  RSAO_CELL_ACTIVE = 1
};

void rsao__rasterize_arrangement_to_grid( rsdb_t* rsdb, msh_array(rs_obj_plcmnt_t) arrangement, isect_grid3d_t* grd );

void
rsao_init_opts( rsao_opts_t* opts )
{
  opts->lower_idx       = 0;
  opts->upper_idx       = 10;
  opts->n_past_steps    = 5;
  opts->n_sa_iter       = 25000;

  opts->energy_function_weights_greedy[ET_SCNCOV] = 2.0;
  opts->energy_function_weights_greedy[ET_GEOM]   = 0.3;
  opts->energy_function_weights_greedy[ET_ISECT]  = 1.0;
  opts->energy_function_weights_greedy[ET_HYSTER] = 1.8;

  opts->energy_function_weights_sa[ET_SCNCOV] = 1.8;
  opts->energy_function_weights_sa[ET_GEOM]   = 0.5;
  opts->energy_function_weights_sa[ET_ISECT]  = 1.0;
  opts->energy_function_weights_sa[ET_HYSTER] = 3.0;

  opts->simulated_annealing_action_likelihoods[ RSAO_ADD ] = 0.01;
  opts->simulated_annealing_action_likelihoods[ RSAO_REM ] = 0.01;
  opts->simulated_annealing_action_likelihoods[ RSAO_REP ] = 0.01;
  opts->simulated_annealing_action_likelihoods[ RSAO_SWP ] = 1.0;
  opts->simulated_annealing_action_likelihoods[ RSAO_MOV ] = 0.5;

  opts->lvl = 3;

  opts->scn_grd = NULL;
}

void
rsao_add_static_objects( rsdb_t* rsdb, int32_t scene_idx )
{
  assert( rsdb );
  assert( scene_idx > 0 );

  for( size_t i = 0; i < msh_array_len( rsdb->arrangements[scene_idx-1]); ++i )
  {
    rs_obj_plcmnt_t* plcmnt = &rsdb->arrangements[scene_idx-1][i];
    if( rsdb_is_object_static(rsdb, plcmnt->object_idx) )
    { 
      msh_array_push( rsdb->arrangements[scene_idx], *plcmnt ); 
    }
  }
}

float
rsao__simulated_annealing_acceptance_function( float prev_score, float score, float temperature)
{
  if ( score > prev_score ) return 1.0;
  else return exp( -( prev_score - score ) / temperature );
}

float
rsao__distance_score( float dist_sq, float dist_sigma )
{
  float inv_denom  = 1.0f / (2.0f * dist_sigma * dist_sigma);
  return ( expf( -dist_sq * inv_denom ) );
}

float
rsao__distance_score( msh_vec3_t p0, msh_vec3_t p1, float dist_sigma )
{
  msh_vec3_t diff = msh_vec3_sub(p1, p0);
  float norm_sq   = msh_vec3_norm_sq(diff);
  return rsao__distance_score( norm_sq, dist_sigma );
}

float
raso__hysteresis_score_sigma_val( float volume )
{
  float x = volume;
  float a = 8.45747072;
  float b = 1.10715163;
  float c = 0.05781275;
  return msh_max( a * expf(-b*x) + c, 0.1f);
}

float
rsao__compute_hysteresis_score( rsdb_t* rsdb, const msh_array(rs_obj_plcmnt_t) arrangement, rsao_opts_t* opts, 
                                bool verbose )
{
  uint64_t t1, t2;
  t1 = msh_time_now();
  float score        = 0.0f;
  int32_t n_obj          = msh_array_len( arrangement );
  int32_t n_arrangements = msh_array_len( rsdb->arrangements );
  if( msh_array_isempty( arrangement ) ) { return 0.0f; }

  float object_scores[1024]    = {0};
  float object_distances[1024] = {0};

  float  object_volume[1024]     = {0};
  int8_t object_existed[1024]    = {0};
  int8_t object_mismatched[1024] = {0}; // class idx does not agree

  float coeff_a = -0.05;
  float coeff_b = 1.05;
  float sigma = 0.0f;
  int32_t n_obj_target = -1;
  for( int32_t past_idx = 1; past_idx <= opts->n_past_steps; ++past_idx )
  {
    int32_t arrgmnt_idx = n_arrangements - (past_idx + 1);
    msh_array(rs_obj_plcmnt_t) past_arrangement = rsdb->arrangements[arrgmnt_idx];
    int32_t n_past_obj = msh_array_len(past_arrangement);
    n_obj_target = msh_max( n_past_obj, n_obj_target );
    
    for( int32_t cur_obj_idx = 0; cur_obj_idx < n_obj; cur_obj_idx++ )
    {
      const rs_obj_plcmnt_t* cur_obj = &arrangement[cur_obj_idx ];
      const int32_t cur_class_idx = rsdb->objects[cur_obj->object_idx].class_idx;
      const float cur_volume = mshgeo_bbox_volume( &rsdb->objects[cur_obj->object_idx].shape->bbox );
      object_volume[cur_obj_idx] = cur_volume;
      sigma = raso__hysteresis_score_sigma_val( cur_volume );
      for( int32_t past_obj_idx = 0; past_obj_idx < n_past_obj; past_obj_idx++ )
      {
        const rs_obj_plcmnt_t* past_obj = &past_arrangement[past_obj_idx];
        const int32_t past_class_idx = rsdb->objects[past_obj->object_idx].class_idx;
        
        if( past_obj->uidx == cur_obj->uidx &&
            cur_class_idx == past_class_idx )
        {
          object_existed[cur_obj_idx] = 1;
          msh_vec3_t cp              = msh_vec4_to_vec3( cur_obj->pose.col[3] );
          msh_vec3_t pp              = msh_vec4_to_vec3( past_obj->pose.col[3] );
          float cur_score            = rsao__distance_score( cp, pp, sigma ) * 0.6f;
          cur_score *= (coeff_a * past_idx + coeff_b);
          if( cur_score > object_scores[cur_obj_idx] )
          {
            object_scores[cur_obj_idx] = cur_score;
            msh_vec3_t diff = msh_vec3_sub( cp, pp );
            float norm_sq   = msh_vec3_norm_sq( diff );
            object_distances[cur_obj_idx] = norm_sq;
          }
        }

        if( past_obj->uidx == cur_obj->uidx && cur_class_idx != past_class_idx )
        {
          object_mismatched[ cur_obj_idx ] = 1;
        }
      }
    }
  }

  // Assign penalties /rewards
  for( int32_t i = 0; i < n_obj; ++i )
  {
    if( object_mismatched[i] ) { object_scores[i] = -0.25f; }

    // Slight preference for existing objects
    if( object_existed[i] ) { object_scores[i] += 0.4f; }
    else                    { object_scores[i] += 0.375f; }

    // Clamp to range
    object_scores[i] = msh_min( 1.0f , object_scores[i] );
  }

  // Count duplicates for same unique id
  int32_t n_duplicates = 0;
  for( int32_t i = 0; i < n_obj-1; ++i )
  {
    int32_t uidx1 = arrangement[i].uidx;
    for( int32_t j = i+1; j < n_obj; ++j )
    {
      int32_t uidx2 = arrangement[j].uidx;
      if( uidx1 == uidx2 ) { n_duplicates++; }
    }
  }

  // Calculate final score -> assign penalty to duplicate objects
  for( int32_t i = 0; i < n_obj; ++i )
  {
    score += object_scores[i];
  }

  score /= (float) n_obj;
  score -= n_duplicates * 5.0f;
  score = msh_max( score, 0.0f );

  t2 = msh_time_now();
  if( verbose )
  {
    printf( "Hysteresis Score:\n");
    for( int32_t i = 0; i < n_obj; ++i )
    {
      const rs_obj_plcmnt_t* plcmnt = &arrangement[i];
      int32_t class_idx = rsdb->objects[plcmnt->object_idx].class_idx;
      char* class_name = rsdb_get_class_name(rsdb, class_idx );
      float cur_volume= object_volume[i];
      float sigma = raso__hysteresis_score_sigma_val( cur_volume );
      printf( "%15s.%02d: %6.4f %6.4f |  %6.4f %6.4f %6.4f \n", class_name, plcmnt->uidx, object_scores[i], 
              sqrtf(object_distances[i]), 0.4 + 0.6*rsao__distance_score(object_distances[i], sigma), 
              cur_volume, sigma );
    }
    printf( "| %d | %fms.\n", n_duplicates, msh_time_diff_ms(t2, t1) );
  }
  
  return score;
}

float
rsao__mahalanobis_score( msh_vec3_t p, msh_vec3_t mu, msh_mat3_t sigma, float scale )
{
  msh_vec3_t p_prime = msh_vec3_sub( p, mu );
  msh_mat3_t sigma_inv = msh_mat3_inverse( sigma );
  float mahalanobis_distance = sqrt( msh_vec3_dot( p_prime, msh_mat3_vec3_mul( sigma_inv, p_prime ) ) );
  float val = exp( -(1.0 / (2*scale*scale)) * mahalanobis_distance );
  return val;
}

float
rsao__compute_intersection_score( rsdb_t* rsdb, msh_array(rs_obj_plcmnt_t) arrangement, rsao_opts_t* opts, 
                                  int32_t verbose )
{

  float score = 1.0f;
  float error = 0.0f;
  float scale = 0.39f;
  int32_t n_obj   = msh_array_len( arrangement );

  int32_t error_count = 0;

  for( int32_t plcmnt_idx_a = 0; plcmnt_idx_a < n_obj; plcmnt_idx_a++ )
  {
    const rs_obj_plcmnt_t* plcmnt_a    = &arrangement[ plcmnt_idx_a ];
    const msh_mat4_t pose_a         = plcmnt_a->pose;
    msh_vec3_t c_a = rs_pointcloud_centroid( rsdb->objects[plcmnt_a->object_idx].shape, 0 );
    msh_mat3_t cov_a = rs_pointcloud_covariance( rsdb->objects[plcmnt_a->object_idx].shape, 0 );

    msh_vec3_t p_a = msh_mat4_vec3_mul( pose_a, c_a, 1);
    msh_mat3_t R_a = msh_mat4_to_mat3( pose_a );
    msh_mat3_t R_at = msh_mat3_transpose( R_a );
    msh_mat3_t sig_a = msh_mat3_mul( R_a, msh_mat3_mul( cov_a, R_at ) );
    float cur_error = 0.0f;
    for( int32_t plcmnt_idx_b = 0; plcmnt_idx_b < n_obj; plcmnt_idx_b++ )
    {
      if( plcmnt_idx_b == plcmnt_idx_a ) { continue; }

      const rs_obj_plcmnt_t* plcmnt_b  = &arrangement[ plcmnt_idx_b ];
      const msh_mat4_t pose_b          = plcmnt_b->pose;

      msh_vec3_t c_b = rs_pointcloud_centroid( rsdb->objects[plcmnt_b->object_idx].shape, 0 );
      msh_mat3_t cov_b = rs_pointcloud_covariance( rsdb->objects[plcmnt_b->object_idx].shape, 0 );

      msh_vec3_t p_b = msh_mat4_vec3_mul( pose_b, c_b, 1);
      msh_mat3_t R_b = msh_mat4_to_mat3( pose_b );
      msh_mat3_t R_bt = msh_mat3_transpose( R_b );
      msh_mat3_t sig_b = msh_mat3_mul( R_b, msh_mat3_mul( cov_b, R_bt ) );

      msh_vec3_t midpoint = msh_vec3_scalar_mul( msh_vec3_add( p_a, p_b ), 0.5f );
      float error_a = rsao__mahalanobis_score( midpoint, p_a, sig_a, scale );
      float error_b = rsao__mahalanobis_score( midpoint, p_b, sig_b, scale );
      float err_val = (error_a+error_b) * 0.5f;
      cur_error = msh_max( cur_error, err_val );
    }

#if 0
    error += cur_error;
    error_count++;
    float final_cur_error = error;
#else
    error = msh_max( error, cur_error );
    float final_cur_error = msh_max( error, cur_error );
#endif
    msh_cprintf( verbose, "Overall error for plcmnt %3d | %12.7f | %12.7f\n", plcmnt_a->uidx, cur_error, 
                                                                              final_cur_error );
  }
  if( error_count ) { error /= error_count; }
  score = 1.0 - error;

  msh_cprintf( verbose, "Intersection score: %f\n", score );
  return score;
}

float
rsao__compute_geometry_score( rsdb_t* rsdb, const msh_array(rs_obj_plcmnt_t) arrangement, rsao_opts_t* opts, 
                              int32_t verbose )
{
  uint64_t t1, t2;
  t1 = msh_time_now();
  float score = 0.0f;
  int32_t n_obj   = msh_array_len( arrangement );
  if( n_obj == 0 ) { return 0.0f; }

  for( int32_t obj_idx = 0; obj_idx < n_obj; obj_idx++ )
  {
    const rs_obj_plcmnt_t* obj= &arrangement[ obj_idx ];
    score += obj->score;
  }
  score /= n_obj;
  t2 = msh_time_now();

  if( verbose )
  {
    printf( "Object Scores: " );
    for( int32_t obj_idx = 0; obj_idx < n_obj; obj_idx++ )
    {
      const rs_obj_plcmnt_t* obj= &arrangement[ obj_idx ];
      printf( "%d %f | ", obj->uidx, obj->score );
    }
    printf( "| %fms.\n", msh_time_diff_ms(t2,t1) );
  }
  return score;

}

float
rsao__compute_scene_coverage_score( rsdb_t* rsdb,
                                    msh_array(rs_obj_plcmnt_t) arrangement,
                                    rsao_opts_t* opts, int32_t verbose )
{
  assert( opts->arrangement_grd );
  assert( opts->scn_grd );
  assert( opts->scn_grd->n_cells == opts->arrangement_grd->n_cells );

  uint64_t t2, t1;
  t1 = msh_time_now();

  int32_t aggrement_count = 0;
  int32_t scn_grd_valid_cells = 0;
  rsao__rasterize_arrangement_to_grid( rsdb, arrangement, opts->arrangement_grd );
  for( int32_t i = 0; i < opts->arrangement_grd->n_cells; ++i )
  {
    uint8_t scn_cell = opts->scn_grd->data[i];
    uint8_t arr_cell = opts->arrangement_grd->data[i];
    if( scn_cell > 0 ) { scn_grd_valid_cells++; }
    if( scn_cell > 0 && arr_cell > 0 ) { aggrement_count++; }
  }

  float score = (float)aggrement_count / (float)scn_grd_valid_cells;
  if( scn_grd_valid_cells == 0 ) { score = 0.0f; }
  t2 = msh_time_now();

  msh_cprintf( verbose, "Coverage score: %f | %d %d | %fms\n", score, scn_grd_valid_cells, aggrement_count, msh_time_diff_ms(t2, t1) );
  return score;
}

float
rsao_compute_scene_alignment_score( rsdb_t* rsdb,
                                    msh_array(rs_obj_plcmnt_t) arrangement,
                                    rsao_opts_t* opts, double* weights, int32_t verbose )
{
  float scncov_score = 0.0f;
  float geom_score = 0.0f;
  float isect_score    = 0.0f;
  float hyster_score   = 0.0f;

  if( weights[ET_GEOM] > 0.0 )   geom_score   = rsao__compute_geometry_score( rsdb, arrangement, opts, verbose );
  if( weights[ET_ISECT] > 0.0 )  isect_score  = rsao__compute_intersection_score( rsdb, arrangement, opts, verbose );
  if( weights[ET_HYSTER] > 0.0 ) hyster_score = rsao__compute_hysteresis_score( rsdb, arrangement, opts, verbose );
  if( weights[ET_SCNCOV] > 0.0 ) scncov_score = rsao__compute_scene_coverage_score( rsdb, arrangement, opts, verbose );
  
  double score      = weights[ET_SCNCOV] * scncov_score +
                      weights[ET_GEOM]   * geom_score +
                      weights[ET_ISECT]  * isect_score +
                      weights[ET_HYSTER] * hyster_score;

  double weight_sum = weights[ET_SCNCOV] + weights[ET_GEOM] + weights[ET_ISECT] + weights[ET_HYSTER];
  
  msh_cprintf( verbose, "Score: %g\n  |  Scn. Cov.: %f; Geom: %f; Isect: %f; Hyster: %f;\n", score / weight_sum, 
              scncov_score, geom_score,
              isect_score, hyster_score );

  return (float)(score / weight_sum);
}

int32_t
rsao__find_max_uidx( rsdb_t* rsdb, int32_t *scene_idx, int32_t *plcmnt_idx )
{
  int32_t max_uidx = 0;
  for( size_t i = 0; i < msh_array_len(rsdb->arrangements); ++i )
  {
    for( size_t j = 0; j < msh_array_len( rsdb->arrangements[i] ); ++j )
    {
      rs_obj_plcmnt_t* obj_plcmnt = &rsdb->arrangements[i][j];
      if( !rsdb_is_object_static( rsdb, obj_plcmnt->object_idx ) && obj_plcmnt->uidx > max_uidx )
      {
        max_uidx = obj_plcmnt->uidx;
        if( scene_idx )  { *scene_idx  = i; }
        if( plcmnt_idx ) { *plcmnt_idx = j; }
      }
    }
  }
  return max_uidx;
}

void
rsao__find_used_uids( msh_array( rs_obj_plcmnt_t) arrangement, msh_map_t* used_ids )
{
  for( size_t i = 0 ; i < msh_array_len(arrangement); ++i )
  {
    uint64_t uidx = arrangement[i].uidx;
    msh_map_insert( used_ids, uidx, uidx );
  }
}

void
rsao__linearize_ids( rsdb_t* rsdb )
{
  // Find largest
  int32_t scene_idx = -1;
  int32_t plcmnt_idx = -1;
  int32_t max_uidx = rsao__find_max_uidx( rsdb, &scene_idx, &plcmnt_idx );

  // Find second largest
  int32_t second_max_uidx = 0;
  for( size_t i = 0; i < msh_array_len(rsdb->arrangements); ++i )
  {
    for( size_t j = 0; j < msh_array_len( rsdb->arrangements[i] ); ++j )
    {
      rs_obj_plcmnt_t* obj_plcmnt = &rsdb->arrangements[i][j];
      if( !rsdb_is_object_static( rsdb, obj_plcmnt->object_idx ) &&
          (int32_t)obj_plcmnt->uidx > second_max_uidx &&
          (int32_t)obj_plcmnt->uidx < max_uidx )
      {
        second_max_uidx = obj_plcmnt->uidx;
      }
    }
  }
  
  // If diff > 1, make it equal 1
  int32_t diff = max_uidx - second_max_uidx;
  if( diff > 1 )
  {
    int32_t new_max_uidx = second_max_uidx + 1;
    rsdb->arrangements[scene_idx][plcmnt_idx].uidx = new_max_uidx;
  }
}

int32_t
rs_obj_plcmnt_cmp( const void *a, const void *b)
{
  rs_obj_plcmnt_t* obs_a = (rs_obj_plcmnt_t*)a;
  rs_obj_plcmnt_t* obs_b = (rs_obj_plcmnt_t*)b;

  return (obs_a->score > obs_b->score) - (obs_a->score < obs_b->score);
}

int32_t
rs_obj_plcmnt_rcmp( const void *a, const void *b)
{
  rs_obj_plcmnt_t* obs_a = (rs_obj_plcmnt_t*)a;
  rs_obj_plcmnt_t* obs_b = (rs_obj_plcmnt_t*)b;

  return (obs_a->score < obs_b->score) - (obs_a->score > obs_b->score);
}

typedef struct rsao_action_info
{
  int32_t placement_idx;
  int32_t plcmnt_idx_a;
  int32_t plcmnt_idx_b;
  rs_obj_plcmnt_t plcmnt_a;
  rs_obj_plcmnt_t plcmnt_b;
} rsao_action_info_t;

rsao_action_info_t
rsao__apply_move_action( rsdb_t* rsdb, int32_t scene_idx, msh_rand_ctx_t* rand_gen, 
                         msh_array(rs_obj_plcmnt_t)* arrangement )
{
  rsao_action_info_t ai = {0};
  msh_array(msh_array(msh_mat4_t)) cur_poses = rsdb->proposed_poses[scene_idx];
  msh_array(msh_array(float)) cur_scores = rsdb->proposed_poses_scores[scene_idx];
  int32_t n_plcmnts = msh_array_len( *arrangement );

  // select object & save info
  int32_t plcmnt_idx = msh_rand_next( rand_gen ) % n_plcmnts;

  ai.plcmnt_idx_a = plcmnt_idx;
  ai.plcmnt_a = (*arrangement)[plcmnt_idx];
  int32_t obj_idx = ai.plcmnt_a.object_idx;

  int32_t n_poses = msh_array_len( cur_poses[obj_idx] );
  int32_t pose_idx = msh_rand_next( rand_gen ) % n_poses;

  rs_obj_plcmnt_t new_plcmnt = ai.plcmnt_a;
  new_plcmnt.pose  = cur_poses[obj_idx][pose_idx];
  new_plcmnt.score = cur_scores[obj_idx][pose_idx];
  (*arrangement)[plcmnt_idx] = new_plcmnt;

  return ai;
}

void
rsao__revert_move_action( const rsao_action_info_t* ai, msh_array(rs_obj_plcmnt_t)* arrangement )
{
  (*arrangement)[ai->plcmnt_idx_a] = ai->plcmnt_a;
}

rsao_action_info_t
rsao__apply_swap_action( rsdb_t* rsdb, int32_t scene_idx, msh_rand_ctx_t* rand_gen,
                         msh_array(rs_obj_plcmnt_t)* arrangement )
{
  rsao_action_info_t ai = {};
  msh_array(msh_array(msh_mat4_t)) cur_poses = rsdb->proposed_poses[scene_idx];
  msh_array(msh_array(float)) cur_scores = rsdb->proposed_poses_scores[scene_idx];
  int32_t n_plcmnts = msh_array_len( *arrangement );

  // select object a
  int32_t idx_a = msh_rand_next( rand_gen ) % n_plcmnts;
  rs_obj_plcmnt_t *plcmnt_a = &(*arrangement)[idx_a];
  int32_t obj_idx_a = plcmnt_a->object_idx;
  msh_mat4_t pose_a = plcmnt_a->pose;

  // select object b
  int32_t idx_b = -1;
  do { idx_b = msh_rand_next( rand_gen ) % n_plcmnts; } while( idx_a == idx_b );
  rs_obj_plcmnt_t *plcmnt_b = &(*arrangement)[idx_b];
  int32_t obj_idx_b = plcmnt_b->object_idx;
  msh_mat4_t pose_b = plcmnt_b->pose;

  // find proposals for a that minimizes distances to b
  float min_dist = 1e9;
  for( size_t i = 0; i < msh_array_len(cur_poses[obj_idx_a]); ++i )
  {
    msh_vec3_t p0 = msh_vec4_to_vec3( pose_b.col[3] );
    msh_vec3_t p1 = msh_vec4_to_vec3( cur_poses[obj_idx_a][i].col[3] );
    msh_vec3_t diff = msh_vec3_sub( p0, p1 );
    float dist = msh_vec3_norm( diff );
    if( dist < min_dist ) { min_dist = dist; }
  }

  float dist_threshold = 0.75;
  if( dist_threshold < min_dist ) { dist_threshold = 2.0 * min_dist; }
  msh_array( int32_t ) a_to_b_ind = {0};
  for( size_t i = 0; i < msh_array_len(cur_poses[obj_idx_a]); ++i )
  {
    msh_vec3_t p0 = msh_vec4_to_vec3( pose_b.col[3] );
    msh_vec3_t p1 = msh_vec4_to_vec3( cur_poses[obj_idx_a][i].col[3] );
    msh_vec3_t diff = msh_vec3_sub( p0, p1 );
    float dist = msh_vec3_norm( diff );
    if( dist < dist_threshold ) { msh_array_push( a_to_b_ind, i ); }
  }

  // find proposals for b that minimizes distances to a
  min_dist = 1e9;
  for( size_t i = 0; i < msh_array_len(cur_poses[obj_idx_b]); ++i )
  {
    msh_vec3_t p0 = msh_vec4_to_vec3( pose_a.col[3] );
    msh_vec3_t p1 = msh_vec4_to_vec3( cur_poses[obj_idx_b][i].col[3] );
    msh_vec3_t diff = msh_vec3_sub( p0, p1 );
    float dist = msh_vec3_norm( diff );
    if( dist < min_dist ) { min_dist = dist; }
  }


  dist_threshold = 0.75;
  if( dist_threshold < min_dist ) { dist_threshold = 2.0 * min_dist; }
  msh_array( int32_t ) b_to_a_ind = {0};
  for( size_t i = 0; i < msh_array_len(cur_poses[obj_idx_b]); ++i )
  {
    msh_vec3_t p0 = msh_vec4_to_vec3( pose_a.col[3] );
    msh_vec3_t p1 = msh_vec4_to_vec3( cur_poses[obj_idx_b][i].col[3] );
    msh_vec3_t diff = msh_vec3_sub(p0, p1);
    float dist = msh_vec3_norm( diff );
    if( dist < dist_threshold ) { msh_array_push( b_to_a_ind, i ); }
  }

  // randomly select from both
  int32_t new_pose_idx_a = a_to_b_ind[ msh_rand_next( rand_gen ) % msh_array_len( a_to_b_ind )];
  int32_t new_pose_idx_b = b_to_a_ind[ msh_rand_next( rand_gen ) % msh_array_len( b_to_a_ind )];

  msh_array_free( a_to_b_ind );
  msh_array_free( b_to_a_ind );

  // store old info for possible reversal
  ai.plcmnt_idx_a = idx_a;
  ai.plcmnt_idx_b = idx_b;
  ai.plcmnt_a = *plcmnt_a;
  ai.plcmnt_b = *plcmnt_b;

  // assign these poses to the placements in arrangements
  plcmnt_a->pose     = cur_poses[obj_idx_a][new_pose_idx_a];
  plcmnt_a->score    = cur_scores[obj_idx_a][new_pose_idx_a];
  plcmnt_a->pose_idx = new_pose_idx_a;

  plcmnt_b->pose     = cur_poses[obj_idx_b][new_pose_idx_b];
  plcmnt_b->score    = cur_scores[obj_idx_b][new_pose_idx_b];
  plcmnt_b->pose_idx = new_pose_idx_b;

  return ai;
}

void
rsao__revert_swap_action( const rsao_action_info_t* ai, msh_array(rs_obj_plcmnt_t)* arrangement )
{
  (*arrangement)[ai->plcmnt_idx_a] = ai->plcmnt_a;
  (*arrangement)[ai->plcmnt_idx_b] = ai->plcmnt_b;
}

rsao_action_info_t
rsao__apply_add_action( rsdb_t* rsdb, int32_t scene_idx, msh_rand_ctx_t* rand_gen,
                        msh_array(rs_obj_plcmnt_t)* arrangement )
{
  rsao_action_info_t ai = {0};

  int32_t max_uidx = rsao__find_max_uidx(rsdb, NULL, NULL );
  msh_map_t used_indices = {0};
  rsao__find_used_uids( rsdb->arrangements[scene_idx], &used_indices );

  // select object
  double object_likelihoods[1024] = {0};
  double object_pdf[1024]         = {0};
  for( size_t i = 0; i < msh_array_len(rsdb->objects); ++i )
  {
    object_likelihoods[i] = 1;
    if( rsdb_is_class_static( rsdb, rsdb->objects[i].class_idx ) )
    {
      object_likelihoods[i] = 0; 
      continue;
    }
  }

  msh_distrib2pdf( object_likelihoods, object_pdf, msh_array_len(rsdb->objects) );
  int32_t obj_idx = msh_pdfsample_linear( object_pdf,
                                      msh_rand_nextf( rand_gen ),
                                      msh_array_len( rsdb->objects ) );

  msh_array(msh_mat4_t) proposed_poses = rsdb->proposed_poses[scene_idx][obj_idx];
  msh_array(float) proposed_scores     = rsdb->proposed_poses_scores[scene_idx][obj_idx];


  double scores[4096] = {0};
  double pdf[4096] = {0};
  for( size_t i = 0; i < msh_array_len(proposed_poses); ++i )
  {
    double score = (double)proposed_scores[i];
    scores[i] = score;
    pdf[i] = 0.0;
  }
  double max_score = 0.0f;
  double min_score = 1.0f;
  for( size_t i = 0; i < msh_array_len(proposed_poses); ++i )
  {
    max_score = msh_max(scores[i], max_score);
    min_score = msh_min(scores[i], min_score);
  }
  double score_diff = max_score - min_score;
  for( size_t i = 0; i < msh_array_len(proposed_poses); ++i )
  {
    scores[i] = (scores[i] - (min_score)) / score_diff;
  }

  msh_distrib2pdf( scores, pdf, msh_array_len(proposed_poses) );

  int32_t selected_idx = msh_pdfsample_linear( pdf, msh_rand_nextf(rand_gen),
                                           msh_array_len(proposed_poses) );

  int32_t uidx = rsdb->objects[obj_idx].uidx;
  uint64_t* used_idx = msh_map_get( &used_indices, (uint64_t)uidx );
  if( used_idx )  { uidx = max_uidx + 1; }
  rs_obj_plcmnt_t plcmnt = { uidx,
                             scene_idx,
                             obj_idx,
                             selected_idx,
                             proposed_poses[selected_idx],
                             proposed_scores[selected_idx] };

  msh_array_push((*arrangement), plcmnt);
  msh_map_free( &used_indices );
  return ai;
}

void
rsao__revert_add_action( const rsao_action_info_t* ai, msh_array(rs_obj_plcmnt_t)* arrangement )
{
  msh_array_pop( *arrangement );
}

rsao_action_info_t
rsao__apply_remove_action( rsdb_t* rsdb, int32_t scene_idx, msh_rand_ctx_t* rand_gen,
                           msh_array(rs_obj_plcmnt_t)* arrangement )
{
  rsao_action_info_t ai = {0};
  int32_t n_obj = msh_array_len( *arrangement );
  ai.placement_idx = msh_rand_next( rand_gen ) % n_obj;
  rs_obj_plcmnt_t tmp = (*arrangement)[ai.placement_idx];
  (*arrangement)[ai.placement_idx] = (*arrangement)[n_obj-1];
  (*arrangement)[n_obj-1] = tmp;
  msh_array_pop(*arrangement);
  return ai;
}

void
rsao__revert_remove_action( const rsao_action_info_t * action_info,
                            msh_array(rs_obj_plcmnt_t)* arrangement )
{
  int32_t n_obj = msh_array_len(*arrangement);
  msh_array_push( (*arrangement), (*arrangement)[n_obj] );
}

rsao_action_info_t
rsao__apply_replace_action( rsdb_t* rsdb, int32_t scene_idx,
                            msh_rand_ctx_t* rand_gen,
                            msh_array(rs_obj_plcmnt_t)* arrangement )
{
  rsao_action_info_t ai = {0};
  ai.plcmnt_idx_a = msh_rand_next( rand_gen) % msh_array_len( *arrangement );
  ai.plcmnt_a = (*arrangement)[ ai.plcmnt_idx_a ];

  // find best possible replacement for it
  msh_vec3_t old_pos = msh_vec4_to_vec3( ai.plcmnt_a.pose.col[3] );
  double scores[1024] = {0};
  int32_t object_ids[1024] = {0};
  int32_t pose_ids[1024] = {0};
  size_t count = 0;
  
  for( size_t i = 0 ; i < msh_array_len(rsdb->objects); ++i )
  {
    if( count >= 1024 ) { printf("%d %d\n", i, (int32_t)msh_array_len(rsdb->objects)); break; }
    if( rsdb_is_object_static(rsdb, i) ) { continue; }
    for( size_t j = 0; j < msh_array_len(  rsdb->proposed_poses[scene_idx][i] ); ++j )
    {
      msh_vec3_t new_pos = msh_vec4_to_vec3( rsdb->proposed_poses[scene_idx][i][j].col[3] );

      if( msh_vec3_norm(msh_vec3_sub(new_pos, old_pos)) < 0.35f )
      {
        object_ids[count] = i;
        pose_ids[count] = j;
        scores[count] = rsdb->proposed_poses_scores[scene_idx][i][j];
        count++;
      }
    }
  }
  if( count < 1 ) { ai.plcmnt_a = (*arrangement)[ ai.plcmnt_idx_a ]; return ai; }

  double score_pdf[1024] = {0};
  msh_distrib2pdf( scores, score_pdf, count);
  int32_t selected_idx = (int32_t)msh_pdfsample_linear( score_pdf, msh_rand_nextf( rand_gen), count);
  int32_t object_idx = object_ids[selected_idx];
  int32_t best_pose_idx = pose_ids[selected_idx];

  // compute and assign new placement
  ai.plcmnt_b = { rsdb->objects[object_idx].uidx,
                  (int32_t)msh_array_len(rsdb->arrangements),
                  object_idx,
                  best_pose_idx,
                  rsdb->proposed_poses[scene_idx][object_idx][best_pose_idx],
                  rsdb->proposed_poses_scores[scene_idx][object_idx][best_pose_idx] };
    
  (*arrangement)[ai.plcmnt_idx_a] = ai.plcmnt_b;
  return ai;
}

void
rsao__revert_replace_action( const rsao_action_info_t * action_info,
                             msh_array(rs_obj_plcmnt_t)* arrangement )
{
  (*arrangement)[action_info->plcmnt_idx_a] = action_info->plcmnt_a;
}

void
rsao_simulated_annealing( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* opts )
{
  uint64_t t1, t2;
  t1 = msh_time_now();
  msh_array(rs_obj_plcmnt_t) cur_arrangement = {0};
  msh_rand_ctx_t rand_gen = {{0}};
  msh_rand_init(&rand_gen, 12346ULL );

  for( size_t i = 0; i < msh_array_len( rsdb->arrangements[scene_idx] ); ++i )
  {
    msh_array_push( cur_arrangement, rsdb->arrangements[scene_idx][i] );
  }

  if( opts->simulated_annealing_action_likelihoods[RSAO_ADD] < 0.001 && 
      msh_array_len( cur_arrangement) < 2 ) 
  {
    return;
  }

  double* weights  = opts->energy_function_weights_sa;
  float init_score = rsao_compute_scene_alignment_score( rsdb, cur_arrangement, opts, weights, 0 );
  float prev_score = init_score;
  float max_score  = init_score;
  float min_temp   = 0.0001f;
  float init_temp  = 0.01f;
  float rand_restart_prob = 0.01f;
  for( int32_t iter = 0; iter < opts->n_sa_iter; iter++ )
  {
    msh_cprintf(iter % 100 == 0, "SIMULATED_ANNEALING: Iteration %d\n", iter );

    // Step 0. Decide if random restart
    if (msh_rand_nextf( &rand_gen ) < rand_restart_prob )
    {
      msh_cprintf(iter%100 == 0,  "SIMULATED_ANNEALING: Random restart!\n" );
      msh_array_clear( cur_arrangement );
      for( size_t i = 0; i < msh_array_len( rsdb->arrangements[scene_idx] ); ++i )
      {
        msh_array_push( cur_arrangement, rsdb->arrangements[scene_idx][i] );
      }
    }

    // Step 1. Decide an action.
    double action_likelihoods[RSAO_N_ACTIONS];
    memmove( action_likelihoods, opts->simulated_annealing_action_likelihoods, sizeof(double) * RSAO_N_ACTIONS );
    if( msh_array_len( cur_arrangement) < 2 ) { action_likelihoods[RSAO_SWP] = 0.0f; }
    double action_pdf[RSAO_N_ACTIONS] = {0};

    msh_distrib2pdf( action_likelihoods, action_pdf, RSAO_N_ACTIONS );

    rsao_action_t action = (rsao_action_t)msh_pdfsample_linear( action_pdf,
                                                                msh_rand_nextf(&rand_gen),
                                                                RSAO_N_ACTIONS );
    // Step 2. Apply random action
    rsao_action_info_t action_info = {0};
    if( action == RSAO_ADD )
    {
      action_info = rsao__apply_add_action( rsdb, scene_idx, &rand_gen, &cur_arrangement );
    }
    else if( action == RSAO_REM )
    {
      action_info = rsao__apply_remove_action( rsdb, scene_idx, &rand_gen, &cur_arrangement );
    }
    else if( action == RSAO_REP )
    {
      action_info = rsao__apply_replace_action( rsdb, scene_idx, &rand_gen, &cur_arrangement );
    }
    else if( action == RSAO_SWP )
    {
      action_info = rsao__apply_swap_action( rsdb, scene_idx, &rand_gen, &cur_arrangement );
    }
    else if( action == RSAO_MOV )
    {
      action_info = rsao__apply_move_action( rsdb, scene_idx, &rand_gen, &cur_arrangement );
    }

    // Step 3. Quantify whether action improved state
    float cur_score = rsao_compute_scene_alignment_score( rsdb, cur_arrangement, opts, weights, 0 );
    if( cur_score < 0 )
    {
      if( msh_array_len(rsdb->arrangements[scene_idx] ) )
        {
          msh_array_clear(rsdb->arrangements[scene_idx]);
        }
        for( size_t ai = 0; ai < msh_array_len(cur_arrangement); ++ai )
        {
          msh_array_push( rsdb->arrangements[scene_idx], cur_arrangement[ai] );
        }
      break;
    }
    float temp = msh_max( init_temp * (1.0f - pow((float)iter / opts->n_sa_iter, 1.0f)), min_temp );
    float acceptance_prob =  rsao__simulated_annealing_acceptance_function( prev_score, cur_score, temp );
    float roll = msh_rand_nextf(&rand_gen);

    msh_cprintf( iter % 100 == 0, "SIMULATED_ANNEALING: Prev. Score: %f Cur. Score: %f Max. Score %f | "
                                  "Prob. %f Roll. %f | Accept: %d | Update: %d | Temp. %f\n",
                                  prev_score, cur_score, max_score,
                                  acceptance_prob, roll,
                                  (bool)(acceptance_prob >= roll),
                                  (bool)(cur_score > max_score),
                                  temp );
    // Step 4. Decide if wish to accept the action
    if( acceptance_prob >= roll )
    {
      prev_score = cur_score;
      if( cur_score > max_score )
      {
        max_score = cur_score;
        if( msh_array_len(rsdb->arrangements[scene_idx] ) )
        {
          msh_array_clear(rsdb->arrangements[scene_idx]);
        }
        for( size_t ai = 0; ai < msh_array_len(cur_arrangement); ++ai )
        {
          msh_array_push( rsdb->arrangements[scene_idx], cur_arrangement[ai] );
        }
      }
    }
    else
    {
      if( action == RSAO_ADD )
      {
        rsao__revert_add_action( &action_info, &cur_arrangement );
      }
      else if( action == RSAO_REM )
      {
        rsao__revert_remove_action( &action_info, &cur_arrangement );
      }
      else if( action == RSAO_REP )
      {
        rsao__revert_replace_action( &action_info, &cur_arrangement );
      }
      else if( action == RSAO_SWP )
      {
        rsao__revert_swap_action( &action_info, &cur_arrangement );
      }
      else if( action == RSAO_MOV )
      {
        rsao__revert_move_action( &action_info, &cur_arrangement );
      }
    }
    rsao__linearize_ids( rsdb );
  }

  t2 = msh_time_now();
  printf( "SIMULATED_ANNEALING: Done in %fs. | Score: %f -> %f\n", msh_time_diff_sec( t2, t1), init_score, max_score );
}

float
rsao_greedy_step( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* opts )
{
  uint64_t t1, t2;
  msh_array( msh_array( msh_mat4_t ) ) cur_poses = rsdb->proposed_poses[scene_idx];
  msh_array( msh_array( float ) ) cur_scores     = rsdb->proposed_poses_scores[scene_idx];
  msh_array( rs_obj_plcmnt_t ) cur_arrangement   = {0};
  opts->proposed_poses = cur_poses;
  
  msh_array( rs_obj_plcmnt_t ) proposals         = {0};

  // Previous score
  double* weights = opts->energy_function_weights_greedy;
  double init_score = rsao_compute_scene_alignment_score( rsdb, rsdb->arrangements[scene_idx], opts, weights, 0 );

  // Gather proposals
  msh_cprintf( 1, "GREEDY STEP: Gathering proposals...\n" );
  t1 = msh_time_now();

  int32_t max_uidx = rsao__find_max_uidx( rsdb, NULL, NULL );

  msh_cprintf( 1, "GREEDY STEP:   Maximum UID so far: %d\n", max_uidx );

  msh_map_t used_indices = {0}; // NOTE(maciej): Actually this should be a set;
  rsao__find_used_uids( rsdb->arrangements[scene_idx], &used_indices );
  msh_cprintf( 1, "GREEDY STEP:   No. of used ids: %d\n", (int32_t)msh_map_len( &used_indices ) );

  for( int32_t obj_idx = 0; obj_idx < (int32_t)msh_array_len( cur_poses ); ++obj_idx )
  {
    if( rsdb_is_object_static( rsdb, obj_idx ) ) { continue; }
    int32_t uidx  = rsdb->objects[obj_idx].uidx;

    for( int32_t pose_idx = 0; pose_idx < (int32_t)msh_array_len(cur_poses[obj_idx]); ++pose_idx )
    {
      rs_obj_plcmnt_t plcmnt = { .uidx            = uidx,
                                 .arrangement_idx = scene_idx,
                                 .object_idx      = obj_idx,
                                 .pose_idx        = pose_idx,
                                 .pose            = cur_poses[obj_idx][pose_idx],
                                 .score           = cur_scores[obj_idx][pose_idx] };
      msh_array_push( proposals, plcmnt );
    }
  }

  t2 = msh_time_now();
  msh_cprintf( 1, "GREEDY STEP:   Got %zu proposals in %fms.\n",
                  msh_array_len(proposals), msh_time_diff_ms( t2, t1) );

  if( msh_array_len( proposals ) <= 0 )
  {
    return rsao_compute_scene_alignment_score( rsdb, rsdb->arrangements[scene_idx], opts, weights, 0 );
  }

  // Copy old state
  for( size_t i = 0 ; i < msh_array_len( rsdb->arrangements[scene_idx] ); ++i )
  {
    msh_array_push( cur_arrangement, rsdb->arrangements[scene_idx][i] );
  }

  // Fix if duplicated and no more objects to use -> this allows for object reuse.
  t1 = msh_time_now();
  for( size_t i = 0 ; i < msh_array_len(proposals); ++i )
  {
    int32_t uidx = proposals[i].uidx;
    uint64_t* used_idx = msh_map_get( &used_indices, (uint64_t)uidx );
    if( used_idx )
    {
      proposals[i].uidx = max_uidx + 1;
    }
  }
  msh_map_free( &used_indices );

  // Score proposals according to a global energy function
  double max_score = 0;
  int32_t max_idx = -1;
  for( size_t i = 0 ; i < msh_array_len(proposals); ++i )
  {
    msh_array_push( cur_arrangement, proposals[i] );
    double cur_score = rsao_compute_scene_alignment_score( rsdb,
                                                          cur_arrangement,
                                                          opts, weights, 0 );
    if( (cur_score - max_score) > RSAO_EPS ) { max_score = cur_score; max_idx = i; }
    msh_array_pop( cur_arrangement );
  }
  msh_cprintf( 1, "GREEDY STEP:   Max score: %f\n", max_score );

  msh_array_push( rsdb->arrangements[scene_idx], proposals[ max_idx ] );

  double score = rsao_compute_scene_alignment_score( rsdb, rsdb->arrangements[scene_idx], opts, weights, 0 );
  t2 = msh_time_now();

  char up[] = "Continue";
  char down[] = "Stop";
  char* sign = ((score - init_score) > RSAO_EPS) ? up : down;
  msh_cprintf( 1, "GREEDY STEP: Finding best proposals took %fs. New score : %f (Delta: %f) | %s\n",
                  msh_time_diff_sec( t2, t1), score, score - init_score, sign );

  return score;
}

void
rsao__rasterize_proposals_to_grid( rsdb_t* rsdb, msh_array(msh_array(msh_mat4_t)) proposed_poses,
                                   isect_grid3d_t* grd, uint8_t cell_value,
                                   int8_t object_type_switch, int32_t lvl )
{
  for( size_t obj_idx = 0; obj_idx < msh_array_len( proposed_poses ); ++obj_idx )
  {
    bool object_is_static = rsdb_is_object_static( rsdb, obj_idx );
    if( object_type_switch == RSAO_DYNAMIC_OBJECT && object_is_static ) { continue; }
    if( object_type_switch == RSAO_STATIC_OBJECT && !object_is_static ) { continue; }
    
    rs_pointcloud_t* obj_pc = rsdb->objects[obj_idx].shape;
    msh_array( msh_mat4_t ) poses = proposed_poses[ obj_idx ];
    if( !poses ) { continue; }

    for( size_t pose_idx = 0; pose_idx < msh_array_len(poses); ++pose_idx )
    {
      for( size_t pt_idx = 0; pt_idx < obj_pc->n_pts[lvl]; ++pt_idx )
      {
        msh_vec3_t world_space_pt = msh_mat4_vec3_mul( poses[pose_idx], obj_pc->positions[lvl][pt_idx], 1 );
        uint8_t *cell = isect_grid3d_cell_from_world_space( grd, world_space_pt );
        if( cell) { *cell = cell_value; }
      }
    }
  }
}

void rsao_rasterize_scene_to_grid( rs_scene_t* scn, isect_grid3d_t* grd, float quality_threshold )
{
  assert(scn);
  assert(scn->shape);
  assert(grd);
  int32_t lvl = 2;
  memset(grd->data, 0, grd->n_cells*sizeof(grd->data[0]));
  for( size_t pt_idx = 0; pt_idx < scn->shape->n_pts[lvl]; ++pt_idx )
  {
    float quality = scn->shape->qualities[lvl][pt_idx];
    if( quality < quality_threshold ) { continue; }
    msh_vec3_t pos = scn->shape->positions[lvl][pt_idx];
    uint8_t* cell = isect_grid3d_cell_from_world_space( grd, pos );
    if( cell ) { *cell = RSAO_CELL_ACTIVE; }
  }
}


void
rsao__rasterize_arrangement_to_grid( rsdb_t* rsdb, msh_array(rs_obj_plcmnt_t) arrangement, 
                                   isect_grid3d_t* grd )
{
  assert( rsdb );
  assert( grd );
  int32_t lvl = 2;
  memset(grd->data, 0, grd->n_cells*sizeof(grd->data[0]));
  for( size_t plcmnt_idx = 0; plcmnt_idx < msh_array_len( arrangement ); ++plcmnt_idx )
  {
    rs_obj_plcmnt_t* plcmnt = &arrangement[plcmnt_idx];
    int32_t obj_idx = plcmnt->object_idx;
    msh_mat4_t pose = plcmnt->pose;
    bool object_is_static = rsdb_is_object_static( rsdb, obj_idx );
    if( object_is_static ) { continue; }
    rs_pointcloud_t* obj_pc = rsdb->objects[obj_idx].shape;

    for( size_t pt_idx = 0; pt_idx < obj_pc->n_pts[lvl]; ++pt_idx )
    {
      msh_vec3_t world_space_pt = msh_mat4_vec3_mul( pose, obj_pc->positions[lvl][pt_idx], 1 );
      uint8_t *cell = isect_grid3d_cell_from_world_space( grd, world_space_pt );
      if( cell) { *cell = RSAO_CELL_ACTIVE; }
    }
  }
}

void
rsao__compute_scene_saliency_grid( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* rsao )
{
  assert( rsao->saliency_grd );
  uint64_t t1, t2;
  int32_t rasterize_lvl = 2;
  int32_t verbose = 1;

  t1 = msh_time_now();

  // Get pose proposals
  msh_array( msh_array( msh_mat4_t ) ) pose_proposals = rsdb->proposed_poses[scene_idx];

  // Initialize grid
  rs_pointcloud_t* scn = rsdb->scenes[scene_idx].shape;
  isect_grid3d_t *grd = rsao->saliency_grd;

  // lit a grid cell where proposal of a dynamic object is
  rsao__rasterize_proposals_to_grid( rsdb, pose_proposals, grd,
                                     RSAO_CELL_ACTIVE, RSAO_DYNAMIC_OBJECT, rasterize_lvl );

  // put out a grid cell where static object is
  rsao__rasterize_proposals_to_grid( rsdb, pose_proposals, grd,
                                     RSAO_CELL_INACTIVE, RSAO_STATIC_OBJECT, rasterize_lvl );
  
  // lit surfels within the lit grid points.
  int32_t lvl = 0;
  int32_t floor_idx = rsdb_get_class_idx( rsdb, "floor" );
  int32_t wall_idx = rsdb_get_class_idx( rsdb, "wall" );
  for( size_t i = 0; i < scn->n_pts[lvl]; ++i )
  {
    msh_vec3_t pt = scn->positions[lvl][i];
    int32_t class_idx = scn->class_ids[lvl][i];
    if( class_idx == wall_idx || class_idx == floor_idx )
    {
      scn->qualities[lvl][i] = 0.0f;
      continue;
    }
    uint8_t *cell = isect_grid3d_cell_from_world_space( grd, pt );
    if( cell && *cell == RSAO_CELL_ACTIVE )
    {
      scn->qualities[lvl][i] = 1.0f;
    }
    else
    {
      scn->qualities[lvl][i] = 0.0f;
    }
    
  }
  t2 = msh_time_now();
  msh_cprintf( verbose, "RSAO_SALIENCY: Computed scene saliency in %fms.\n", 
                        msh_time_diff_ms(t2, t1) );
}

void
rsao__compute_scene_saliency_kdtree( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* rsao )
{
  uint64_t t1, t2;
  int32_t lvl = 0;
  int32_t max_n_neigh = 32;
  int32_t verbose = 1;
  float radius = 0.05;
  rs_pointcloud_t* scn_pc = rsdb->scenes[scene_idx].shape;
  msh_hash_grid_t scn_search_index = {0};
  msh_hash_grid_init_3d( &scn_search_index, &scn_pc->positions[lvl][0].x, scn_pc->n_pts[lvl], radius );
  msh_array( msh_array( msh_mat4_t ) ) cur_poses = rsdb->proposed_poses[scene_idx];

  // Clear qualities
  memset( scn_pc->qualities[0], 0, scn_pc->n_pts[0] * sizeof( scn_pc->qualities[0][0]) );

  // Mark where dynamic objects go
  t1 = msh_time_now();
  for( size_t obj_idx = 0; obj_idx < msh_array_len( cur_poses ); ++obj_idx )
  {
    if( rsdb_is_object_static( rsdb, obj_idx ) ) { continue; }
    else
    {
      rs_pointcloud_t* obj_pc = rsdb->objects[obj_idx].shape;

      msh_array( msh_mat4_t ) poses = cur_poses[ obj_idx ];
      if( !poses ) { continue; }

      msh_vec3_t* query_pos = (msh_vec3_t*)malloc( obj_pc->n_pts[lvl] * sizeof(msh_vec3_t) );
      float* nn_dists = (float*)malloc( obj_pc->n_pts[lvl] * max_n_neigh * sizeof(float) );
      int32_t* nn_inds = (int32_t*)malloc( obj_pc->n_pts[lvl] * max_n_neigh * sizeof(int32_t) );
      size_t* n_neighbors = (size_t*)malloc( obj_pc->n_pts[lvl] * sizeof(size_t) );

      for( size_t pose_idx = 0; pose_idx < msh_array_len(poses); ++pose_idx )
      {
        for( size_t pt_idx = 0; pt_idx < obj_pc->n_pts[lvl]; ++pt_idx )
        {
          query_pos[pt_idx] = msh_mat4_vec3_mul( poses[pose_idx], 
                                                 obj_pc->positions[lvl][pt_idx], 1 );
        }

        // Perform the search
        msh_hash_grid_search_desc_t search_opts = {0};
        search_opts.query_pts    = (real32_t*)&query_pos[0].x;
        search_opts.n_query_pts  = obj_pc->n_pts[lvl];
        search_opts.distances_sq = nn_dists;
        search_opts.indices      = nn_inds;
        search_opts.n_neighbors  = n_neighbors;
        search_opts.radius       = radius;
        search_opts.max_n_neigh  = max_n_neigh;

        msh_hash_grid_radius_search( &scn_search_index, &search_opts);
      
        for( size_t pt_idx = 0; pt_idx < obj_pc->n_pts[lvl]; ++pt_idx )
        {
          for( size_t nn_idx = 0; nn_idx < n_neighbors[pt_idx]; ++nn_idx )
          {
            int32_t scn_pt_idx = nn_inds[pt_idx * max_n_neigh + nn_idx];
            scn_pc->qualities[lvl][scn_pt_idx] = 1.0;
          }
        }
      }

      free( query_pos );
      free( nn_dists );
      free( nn_inds );
      free( n_neighbors );
    }
  }

  // Unmark positions where static objects go
  for( size_t obj_idx = 0; obj_idx < msh_array_len( cur_poses ); ++obj_idx )
  {
    if( rsdb_is_object_static( rsdb, obj_idx ) ) 
    {
      rs_object_t* object = &rsdb->objects[obj_idx];
      rs_pointcloud_t* obj_pc = object->shape;

      msh_array( msh_mat4_t ) poses = cur_poses[ obj_idx ];
      if( !poses ) { continue; }

      msh_vec3_t* query_pos = (msh_vec3_t*)malloc( obj_pc->n_pts[lvl] * sizeof(msh_vec3_t) );
      float* nn_dists = (float*)malloc( obj_pc->n_pts[lvl] * max_n_neigh * sizeof(float) );
      int32_t* nn_inds = (int32_t*)malloc( obj_pc->n_pts[lvl] * max_n_neigh * sizeof(int32_t) );
      size_t* n_neighbors = (size_t*)malloc( obj_pc->n_pts[lvl] * sizeof(size_t) );

      for( size_t pose_idx = 0; pose_idx < msh_array_len(poses); ++pose_idx )
      {
        for( size_t pt_idx = 0; pt_idx < obj_pc->n_pts[lvl]; ++pt_idx )
        {
          query_pos[pt_idx] = msh_mat4_vec3_mul( poses[pose_idx], 
                                                 obj_pc->positions[lvl][pt_idx], 1 );
        }

        // Perform the search
        msh_hash_grid_search_desc_t search_opts = {0};
        search_opts.query_pts    = (real32_t*)&query_pos[0].x;
        search_opts.n_query_pts  = obj_pc->n_pts[lvl];
        search_opts.distances_sq = nn_dists;
        search_opts.indices      = nn_inds;
        search_opts.n_neighbors  = n_neighbors;
        search_opts.radius       = radius;
        search_opts.max_n_neigh  = max_n_neigh;

        msh_hash_grid_radius_search( &scn_search_index, &search_opts);
      
        for( size_t pt_idx = 0; pt_idx < obj_pc->n_pts[lvl]; ++pt_idx )
        {
          for( size_t nn_idx = 0; nn_idx < n_neighbors[pt_idx]; ++nn_idx )
          {
            int32_t scn_pt_idx = nn_inds[pt_idx * max_n_neigh + nn_idx];
            scn_pc->qualities[lvl][scn_pt_idx] = 0.0f;
          }
        }
      }

      free( query_pos );
      free( nn_dists );
      free( nn_inds );
      free( n_neighbors );
    }
  }

  msh_hash_grid_term( &scn_search_index );
  t2 = msh_time_now();
  msh_cprintf( verbose, "IO: Time to compute qualities: %fms.\n",
               msh_time_diff_ms( t2, t1) );
}

void
rsao_compute_scene_saliency( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* rsao )
{
  rsao__compute_scene_saliency_grid( rsdb, scene_idx, rsao );
}
