#ifndef RS_ARRANGEMENT_OPTIMIZATION_H
#define RS_ARRANGEMENT_OPTIMIZATION_H


typedef enum rsao_error_term
{
  ET_SCNCOV = 0,
  ET_GEOM,
  ET_ISECT,
  ET_HYSTER,
  ET_N_TERMS
} rsao_error_term_t;

typedef enum rsao_action
{
  RSAO_ADD,
  RSAO_REM,
  RSAO_REP,
  RSAO_SWP,
  RSAO_MOV,
  RSAO_N_ACTIONS
} rsao_action_t;

typedef struct rsao_options
{
  int32_t lower_idx;
  int32_t upper_idx;
  int32_t n_sa_iter;
  int32_t n_past_steps;
  
  double energy_function_weights_sa[ET_N_TERMS];
  double energy_function_weights_greedy[ET_N_TERMS];
  double simulated_annealing_action_likelihoods[RSAO_N_ACTIONS];
 
  int32_t lvl;
  int32_t verbose;

  isect_grid3d_t* scn_grd;
  isect_grid3d_t* arrangement_grd;
  isect_grid3d_t* isect_grd;
  isect_grid3d_t* saliency_grd;
  msh_array(msh_array(msh_mat4_t)) proposed_poses;
} rsao_opts_t;


void
rsao_init_opts( rsao_opts_t* opts );

float
rsao_compute_scene_alignment_score( rsdb_t* rsdb, msh_array(rs_obj_plcmnt_t) arrangement,
                                    rsao_opts_t* opts, double* weights, int32_t verbose );

float
rsao_greedy_step( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* opts );

void
rsao_assign_identities( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* opts );

void 
rsao_simulated_annealing( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* opts );

void
rsao_compute_scene_saliency( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* rsao );

void
rsao_classify_by_pose( rsdb_t* rsdb, int32_t scene_idx, rsao_opts_t* opts );

void 
rsao_rasterize_scene_to_grid( rs_scene_t* scn, isect_grid3d_t* grd, float quality_threshold );

void
rsao_add_static_objects( rsdb_t* rsdb, int32_t scene_idx );


#endif /*RS_ARRANGEMENT_OPTIMIZATION_H*/