
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
#include "msh/msh_ply.h"
#include "rs_pointcloud.h"
#include "rs_distance_function.h"
#include "rs_database.h"

#define INTERSECTION_IMPLEMENTATION
#include "intersect.h"

#include "pose_proposal.h"

void
mgs_init_opts( mgs_opts_t* opts )
{
  opts->search_grid_spacing     = 0.10f;
  opts->search_grid_angle_delta = MSH_TWO_PI / 10.0f;
  opts->max_neigh_search_radius = 0.25f;
  opts->naive                   = false;

  opts->use_geometry            = 1;
  opts->use_mask_rcnn           = 0; 
}

inline double
mgs__exp_score_sq( double dist_sq, double sigma )
{
  return exp( -dist_sq / (2.0*sigma*sigma) );
}

inline float
mgs_weighted_pt2pt_score( msh_vec3_t p, msh_vec3_t q, 
                           msh_vec3_t n, msh_vec3_t m, float factor )
{
  msh_vec3_t diff = msh_vec3_sub( p, q );
  float dist_sq = msh_vec3_norm_sq(diff);
  float dot = msh_vec3_dot(n,m);
  float w = msh_clamp01( dot );
  return w * mgs__exp_score_sq( dist_sq, factor );
}

inline float
mgs_pt2pl_score( msh_vec3_t p, msh_vec3_t q, 
                  msh_vec3_t n, msh_vec3_t m, float factor )
{
  msh_vec3_t diff = msh_vec3_sub( q, p );
  float dist = msh_vec3_dot(n, diff);
  return mgs__exp_score_sq( dist*dist, factor );
}


tmp_score_calc_storage_t
allocate_tmp_calc_storage( const int32_t obj_n_pts, const int32_t scn_n_pts, const int32_t max_n_neigh )
{
  tmp_score_calc_storage_t storage = { 0 };
  storage.max_n_neigh = max_n_neigh;
  storage.obj_pos   = (msh_vec3_t*)malloc( obj_n_pts * sizeof(msh_vec3_t) );
  storage.obj_nor   = (msh_vec3_t*)malloc( obj_n_pts * sizeof(msh_vec3_t) );
  storage.obj_n_pts   = obj_n_pts;

  storage.obj2scn_indices     = (int32_t*)malloc( max_n_neigh * obj_n_pts * sizeof(int32_t) );
  storage.obj2scn_dists_sq    = (float*)malloc( max_n_neigh * obj_n_pts * sizeof(float) );
  storage.obj2scn_n_neighbors = (size_t*)malloc( obj_n_pts * sizeof(size_t) );

  return storage;
}

void
free_tmp_calc_storage( tmp_score_calc_storage_t* storage )
{
  free( storage->obj2scn_indices );
  free( storage->obj2scn_n_neighbors );
  free( storage->obj2scn_dists_sq );

  free( storage->obj_pos );
  free( storage->obj_nor );

  storage->max_n_neigh = 0;
  storage->obj_n_pts = 0;
}

float 
mgs_compute_object_alignment_score( rs_pointcloud_t* object, rs_pointcloud_t* scene, 
                                    int32_t search_lvl, int32_t query_lvl, 
                                    msh_mat4_t xform, tmp_score_calc_storage_t* storage )
{
  static float search_radii[5] = { 0.05f, 0.1f, 0.15f, 0.2f, 0.25f };
  double max_angle = msh_deg2rad( 35.0 );
  double score_sigma = search_radii[search_lvl];
  msh_hash_grid_t* scn_search_index = scene->search_grids[search_lvl];
  double alpha = 0.05;
  double beta  = 1.0 - alpha;

  // Transform object's points
  for( int32_t i = 0; i < storage->obj_n_pts; ++i )
  { 
    msh_vec3_t p = object->positions[query_lvl][i];
    msh_vec3_t n = object->normals[query_lvl][i];
    storage->obj_pos[i] = msh_mat4_vec3_mul( xform, p, 1 );
    storage->obj_nor[i] = msh_mat4_vec3_mul( xform, n, 0 );
  }

  // Perform nearest neighbor search
  msh_hash_grid_search_desc_t search_opts = {0};
  search_opts.query_pts    = &storage->obj_pos[0].x;
  search_opts.n_query_pts  = storage->obj_n_pts;
  search_opts.distances_sq = storage->obj2scn_dists_sq;
  search_opts.indices      = storage->obj2scn_indices;
  search_opts.n_neighbors  = storage->obj2scn_n_neighbors;
  search_opts.radius       = search_radii[search_lvl];
  search_opts.max_n_neigh  = storage->max_n_neigh;
  search_opts.sort         = 1;
  msh_hash_grid_radius_search( scn_search_index, &search_opts );

  double overall_score = 0.0;
  for( int32_t i = 0; i < storage->obj_n_pts; ++i )
  {
    size_t cur_nn = storage->obj2scn_n_neighbors[i];
    double best_dist_sq = -1.0;
    double dot = 0.0;
    double best_angle = 0.0;
    for( size_t j = 0 ; j < cur_nn; ++j )
    {
      int32_t k = storage->obj2scn_indices[ i * storage->max_n_neigh + j ];
      msh_vec3_t n = storage->obj_nor[i];
      msh_vec3_t m = scene->normals[search_lvl][k];
      dot = msh_vec3_dot( m, n );
      dot = msh_max( dot, 0.0f );
      double angle = acos( dot );
      if( angle - max_angle < 0.000001 )
      {
        best_dist_sq = storage->obj2scn_dists_sq[ i * storage->max_n_neigh + j ];
        best_angle = angle;
        break;
      }
    }
    if( best_dist_sq < -0.0001 ) { continue; }
    double normals_compat = exp( -(best_angle*best_angle) / (2.0 * 0.5 * 0.5 ) ); 
    double dist_compat    = mgs__exp_score_sq( best_dist_sq, score_sigma );
    double score = alpha * normals_compat + beta * dist_compat;
    overall_score += score;
  }


  overall_score /= (double)( storage->obj_n_pts );
  return (float)overall_score;
}

float
mgs__score_threshold( int32_t lvl )
{
  if( lvl == RSPC_N_LEVELS - 1 ) { return 0.25f; }
  if( lvl == RSPC_N_LEVELS - 2 ) { return 0.35f; }
  if( lvl == RSPC_N_LEVELS - 3 ) { return 0.40f; }
  if( lvl == RSPC_N_LEVELS - 4 ) { return 0.50f; }
  else                           { return 0.50f; }
}

void
mgs__initial_pose_proposals( rsdb_t* rsdb, rs_pointcloud_t* input_scan, 
                             rs_df_t* df_input_scan, int32_t lvl,
                             msh_array(msh_array(pose_proposal_t)) *proposed_poses, 
                             const mgs_opts_t* opts )
{
  uint64_t st, et;
  int32_t n_objects = msh_array_len( rsdb->objects );
  int32_t search_lvl = 1;
  int32_t max_n_neigh = 64;

  for( int32_t i = 0 ; i < n_objects ; ++i )
  {
    msh_array_push( *proposed_poses, NULL );
  }
  
  printf( "POSE_PROPOSAL:   Starting initial pose proposal search...\n");
  st = msh_time_now(); 
  for( int32_t i = 0 ; i < n_objects; ++i )
  {
    uint64_t t1, t2;
    t1 = msh_time_now();
    
    msh_vec3_t origin   = input_scan->bbox.min_p;
    rs_object_t* object = &rsdb->objects[i];
    tmp_score_calc_storage_t storage = allocate_tmp_calc_storage( object->shape->n_pts[lvl], input_scan->n_pts[lvl], max_n_neigh );
    
    // Skip static
    if( rsdb_is_object_static(rsdb, i) )
    {
      continue;
    }

    float spacing         = opts->search_grid_spacing;
    float y_angle_inc     = opts->search_grid_angle_delta;
    float length_x        = input_scan->bbox.max_p.x - input_scan->bbox.min_p.x;
    float length_z        = input_scan->bbox.max_p.z - input_scan->bbox.min_p.z;
    float height          = 0.0f;
    char* class_name      = rsdb_get_class_name(  rsdb, object->class_idx );
    printf( "POSE_PROPOSAL:      Searching for transformation for model %s.%03d (%d)...\n", 
                               class_name, object->uidx, i );
    float max_score = -1e9;
    float score_threshold = mgs__score_threshold(lvl);
    for(float ox = -spacing; ox < length_x + spacing; ox += spacing )
    {
      for(float oz = -spacing; oz < length_z + spacing; oz += spacing)
      {
        float best_rot_score = 0;
        msh_mat4_t best_rot_xform = {0};
        for( float y_angle = 0.0f; y_angle < MSH_TWO_PI; y_angle += y_angle_inc )
        { 
          msh_mat4_t xform = msh_rotate(msh_mat4_identity(), y_angle, msh_vec3(0.0f, 1.0f, 0.0f));
          xform.col[3] = msh_vec4(origin.x + ox, height, origin.z + oz, 1.0f); 
          if( df_input_scan )
          {
            float nearest = rs_df_closest_surface(df_input_scan, (float*)&xform.col[3]);
            if( nearest > 0.6 ) { continue; }
          }
          float score = mgs_compute_object_alignment_score( object->shape, input_scan, 
                                                            search_lvl, lvl, xform, &storage );
          if( score > best_rot_score ) 
          { 
            best_rot_score = score; 
            best_rot_xform = xform; 
            if( best_rot_score > max_score ) { max_score = best_rot_score; }
          }
        }

        if( best_rot_score > score_threshold )
        {
          pose_proposal_t proposal = {.xform = best_rot_xform, .score = best_rot_score };
          msh_array_push( (*proposed_poses)[i], proposal );
        }
      }
    }
    free_tmp_calc_storage( &storage );
    t2 = msh_time_now();
    double elapsed = msh_time_diff_sec( t2, t1 );
    printf( "POSE_PROPOSAL:         --> Found %zu potential poses in %fs. (Max score: %f)\n", 
                                msh_array_len((*proposed_poses)[i]), elapsed, max_score );
  }
  
  et = msh_time_now();
  printf( "POSE_PROPOSAL:   Initial pose proposals made in %fs.\n", msh_time_diff_sec( et, st) );
}

void
mgs__pose_verification( rsdb_t* rsdb, rs_pointcloud_t* input_scan, int32_t lvl,
                         msh_array(msh_array(pose_proposal_t)) *proposed_poses, 
                         const mgs_opts_t* opts )
{
  float score_threshold = mgs__score_threshold(lvl);
  int32_t n_objects = msh_array_len(rsdb->objects);
  int32_t search_lvl = 1;
  int32_t max_n_neigh = 64;

  for( int32_t i = 0; i < n_objects; ++i )
  {
    if( rsdb_is_object_static( rsdb, i ) ) { continue; }

    int32_t n_poses = msh_array_len((*proposed_poses)[i]);
    int32_t n_valid_poses = n_poses;  
    int32_t class_idx = rsdb->objects[i].class_idx;
    int32_t instance_idx = rsdb->objects[i].uidx;
    tmp_score_calc_storage_t storage = allocate_tmp_calc_storage( rsdb->objects[i].shape->n_pts[lvl], 
                                                                  input_scan->n_pts[lvl], 
                                                                  max_n_neigh );
    if( n_poses )
    {
      char* class_name = rsdb_get_class_name( rsdb, class_idx );
      printf( "POSE_PROPOSAL:      Verifying transformations for model %s.%03d(%d) using...\n", 
              class_name, instance_idx, i);
      float max_score = -1e9;
      for( int32_t j = 0; j < n_poses; j++ )
      {
        if( (*proposed_poses)[i][j].score > 0.0f )
        {
          msh_mat4_t xform = (*proposed_poses)[i][j].xform;
          float score = mgs_compute_object_alignment_score( rsdb->objects[i].shape, input_scan,
                                                            search_lvl, lvl, xform, &storage );
          if(score > max_score )       { max_score = score; }
          if(score > score_threshold ) { (*proposed_poses)[i][j].score = score; }
          else                         { (*proposed_poses)[i][j].score = -1.0f; n_valid_poses--; }
        }
        else
        {
          n_valid_poses--;
        }
      }
      printf( "POSE_PROPOSAL:         --> Found %d poses( Max. score: %f)\n", n_valid_poses, max_score);
    }
    free_tmp_calc_storage(&storage);
  }
}

void
mgs__propose_poses_at_level(rsdb_t* rsdb, rs_pointcloud_t* input_scan, 
                            rs_df_t* df_input_scan, int32_t lvl,
                            msh_array(msh_array(pose_proposal_t)) *proposed_poses, 
                            const mgs_opts_t* opts )
{

  if( !(*proposed_poses) )
  {
    mgs__initial_pose_proposals( rsdb, input_scan, df_input_scan, lvl,
                                 proposed_poses, opts );
  }
  else
  {
    mgs__pose_verification( rsdb, input_scan, lvl,
                             proposed_poses, opts );
  }
} 


void
mgs_propose_poses( rsdb_t* rsdb, rs_pointcloud_t* input_scan, 
                    msh_array(msh_array(pose_proposal_t)) *proposed_poses, 
                    const mgs_opts_t* opts, int32_t verbose )
{
  double st, et;
  double gst, get;
  gst = msh_time_now();
  
  msh_array(msh_array(pose_proposal_t)) proposal_storage = NULL;

  // Pose proposals
  for( int32_t lvl = RSPC_N_LEVELS-1; lvl > RSPC_N_LEVELS-4; lvl--)
  {
    st = msh_time_now();
    msh_cprintf( verbose, "POSE PROPOSAL: Working on level: %d | Threshold: %6.4f\n", lvl, mgs__score_threshold( lvl ) );
    mgs__propose_poses_at_level( rsdb, input_scan, NULL, lvl,
                                 &proposal_storage, opts );
    et = msh_time_now();
    msh_cprintf( verbose, "POSE PROPOSAL: Level %d processing time: %fs\n", lvl, msh_time_diff_sec( et, st)  );
  }

  // Copy valid poses
  for( size_t i = 0; i < msh_array_len( proposal_storage ); ++i )
  {
    msh_array(pose_proposal_t) cur_proposals  = NULL;
    for( size_t j = 0; j < msh_array_len( proposal_storage[i] ); ++j)
    {
      if( fabsf(proposal_storage[i][j].score) > 0.000001f )
      {
        msh_array_push( cur_proposals, proposal_storage[i][j] );
      }
    }
    msh_array_push( *proposed_poses, cur_proposals );
  }

  // Cleanup
  for( size_t i = 0; i < msh_array_len( proposal_storage ); ++i )
  {
    if( proposal_storage[i] ) { msh_array_free(proposal_storage[i]); }
  }
  if( proposal_storage ) { msh_array_free( proposal_storage ); }
  get = msh_time_now();
  msh_cprintf( verbose, "POSE PROPOSAL: Done in %fs\n", msh_time_diff_sec( get, gst ) );
}

void
mgs_non_maxima_suppresion( rsdb_t* rsdb,
                           msh_array(msh_array(pose_proposal_t))* proposed_poses,
                           int32_t verbose, float dist_threshold )
{
  int32_t n_objects = msh_array_len(*proposed_poses);
  for( int32_t i = 0; i < n_objects; ++i )
  {
    // Initialize useful vars
    rs_object_t* object = &rsdb->objects[i];
    rs_pointcloud_t* shape = object->shape;
    msh_vec3_t c = rs_pointcloud_centroid( shape, 0 );
    msh_array(pose_proposal_t) cur_proposals  = (*proposed_poses)[i];
    int32_t n_detections = msh_array_len(cur_proposals);
    if( n_detections == 0 ) { continue; }

    msh_array( mark_t ) indices_marks = {0};
    int32_t marked_detections = 0;
    char* class_name = rsdb_get_class_name( rsdb, object->class_idx );
    msh_cprintf( verbose, "POSE_PROPOSAL: Non-max suppress. : Working on object class %s.%03d (%d/%d) \n",  
                 class_name, object->uidx, i, n_objects );
    for( int32_t i = 0; i < n_detections; ++i ) { msh_array_push( indices_marks, PP_UNMARKED ); }
    while( marked_detections != n_detections )
    {
      // find max unmarked detection
      int32_t max_idx = -1; float max_score = -1e9;
      for( int32_t i = 0 ; i < n_detections ; ++i ) 
      { 
        if( indices_marks[i] == PP_UNMARKED && 
            cur_proposals[i].score > max_score ) 
            { 
              max_score = cur_proposals[i].score; 
              max_idx = i; 
            }
      }
      // mark detection as a keep
      indices_marks[max_idx] = PP_KEEP;
      marked_detections++;
      
      // discard all detections that are overlapping this one.
      for( int32_t i = 0 ; i < n_detections; ++i )
      {
        if( indices_marks[i] == PP_UNMARKED )
        {
          msh_vec3_t p1 = msh_mat4_vec3_mul( cur_proposals[max_idx].xform, c, 1 );
          msh_vec3_t p2 = msh_mat4_vec3_mul( cur_proposals[i].xform, c, 1 );

          float dist = msh_vec3_norm( msh_vec3_sub( p1, p2 ) );
          float overlap = isect_get_overlap_factor(shape, cur_proposals[max_idx].xform,
                                                   shape, cur_proposals[i].xform,
                                                   0.1f, 1, 0 );
          if( overlap > 0.5f || dist < dist_threshold || cur_proposals[i].score < 0.01f )
          {
            indices_marks[i] = PP_DISCARD;
            marked_detections++;
          }
        }
      }
    }

    // report stats
    int32_t counts[3] = {0, 0, 0};
    for( int32_t i = 0 ; i < n_detections; ++i )
    {
      counts[indices_marks[i]]++;
    }
    msh_cprintf( verbose, "POSE_PROPOSAL: Non-max suppress. --> Keep: %5d Discard: %5d Unmarked: %5d\n",
                                       counts[1], counts[2], counts[0]);
    // copy the data
    msh_array(pose_proposal_t) new_proposals = {0};
    for( int32_t i = 0 ; i < n_detections; ++i )
    {
      if( indices_marks[i] == PP_KEEP )
      {
        msh_array_push( new_proposals, cur_proposals[i] );
      }
    }
    (*proposed_poses)[i] = new_proposals;
    msh_array_free( cur_proposals );
    msh_array_free( indices_marks );
  }
}


int32_t
pose_proposal_cmp(const void *a, const void *b)
{
  float score_a = ((pose_proposal_t*)a)->score;
  float score_b = ((pose_proposal_t*)b)->score;
  return (score_a > score_b) ? -1 : (score_a < score_b);
}

void 
mgs_sort_poses( msh_array(msh_array(pose_proposal_t))* poses, int32_t verbose )
{
  uint64_t t1, t2;
  t1 = msh_time_now();
  for(size_t i = 0; i < msh_array_len((*poses)); ++i)
  {
    size_t cur_size = msh_array_len((*poses)[i]);
    qsort( (*poses)[i], cur_size, sizeof(pose_proposal), pose_proposal_cmp );
  }
  t2 = msh_time_now();
  msh_cprintf( verbose, "POSE_PROPOSAL: Sorting poses done in %fms.\n", msh_time_diff_ms(t2,t1));
}