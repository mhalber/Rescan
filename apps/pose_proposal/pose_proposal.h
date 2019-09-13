#ifndef RS_POSE_PROPOSALS_H
#define RS_POSE_PROPOSALS_H

typedef enum mark { PP_UNMARKED=0, PP_KEEP, PP_DISCARD } mark_t;

typedef struct pose_proposal_options_t
{
  float search_grid_spacing;
  float search_grid_angle_delta;
  float max_neigh_search_radius;
  bool naive;
  bool use_geometry;
  bool use_mask_rcnn;
} mgs_opts_t;

typedef struct pose_proposal
{
  msh_mat4_t xform;
  float score;
} pose_proposal_t;

typedef struct tmp_score_calc_storage
{
  int32_t max_n_neigh;

  msh_vec3_t* obj_pos;
  msh_vec3_t* obj_nor;
  int32_t obj_n_pts;

  int32_t *obj2scn_indices;
  size_t *obj2scn_n_neighbors;
  float *obj2scn_dists_sq;

} tmp_score_calc_storage_t;

tmp_score_calc_storage_t
allocate_tmp_calc_storage( const int32_t obj_n_pts, const int32_t scn_n_pts, const int32_t max_n_neigh );

void
free_tmp_calc_storage( tmp_score_calc_storage_t* storage );

void 
mgs_init_opts( mgs_opts_t* opts );

float 
mgs_compute_object_alignment_score(rs_pointcloud_t* object, rs_pointcloud_t* scene, 
                                   int search_lvl, int query_lvl,
                                   msh_mat4_t xform, tmp_score_calc_storage_t* storage );

void
mgs_propose_poses( rsdb_t* rsdb, rs_pointcloud_t* input_scan, 
                    msh_array(msh_array(pose_proposal_t)) *proposed_poses, 
                    const mgs_opts_t* opts, int verbose );
void
mgs_non_maxima_suppresion( rsdb_t* rsdb,
                            msh_array(msh_array(pose_proposal_t))* proposed_poses,
                            int verbose, float dist_threshold );

void 
mgs_sort_poses( msh_array(msh_array(pose_proposal_t))* proposed_poses, 
                 int verbose );

#endif /*RS_POSE_PROPOSALS*/