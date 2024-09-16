#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "mujoco/mujoco.h"

#include <emscripten/fetch.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <vector>

using namespace emscripten;

int finish(const char *msg = NULL, mjModel *m = NULL) {
  if (m  ) { mj_deleteModel(m); }
  if (msg) { std::printf("%s\n", msg); }
  return 0;
}

class Model {
public:
  Model() { m = NULL; }
  Model(const std::string filename) {
    if(0 == filename.compare(filename.length() - 3, 3, "mjb")){
      char error[1000] = "Could not load mjb model";
      m = mj_loadModel(filename.c_str(), 0); 
      if (!m) { finish(error, m); }
    } else {
      char error[1000] = "Could not load xml model";
      m = mj_loadXML(filename.c_str(), 0, error, 1000); 
      if (!m) { finish(error, m); }
    }
  }

  static Model load_from_xml(const std::string filename) { return Model(filename); }
  static Model load_from_mjb(const std::string filename) { return Model(filename); }

  mjModel *ptr       () { return m; }
  mjModel getVal     () { return *m; }
  mjOption getOptions() { return (*m).opt; }
  void free          () { return mju_free(m); }

  // MJMODEL_DEFINITIONS
val  qpos0() const { return val(typed_memory_view(m->nq, m->qpos0)); }
val  qpos_spring() const { return val(typed_memory_view(m->nq, m->qpos_spring)); }
val  body_treeid() const { return val(typed_memory_view(m->nbody, m->body_treeid)); }
val  body_simple() const { return val(typed_memory_view(m->nbody, m->body_simple)); }
val  body_sameframe() const { return val(typed_memory_view(m->nbody, m->body_sameframe)); }
val  body_pos() const { return val(typed_memory_view(m->nbody, m->body_pos)); }
val  body_quat() const { return val(typed_memory_view(m->nbody, m->body_quat)); }
val  body_ipos() const { return val(typed_memory_view(m->nbody, m->body_ipos)); }
val  body_subtreemass() const { return val(typed_memory_view(m->nbody, m->body_subtreemass)); }
val  body_invweight0() const { return val(typed_memory_view(m->nbody, m->body_invweight0)); }
val  body_gravcomp() const { return val(typed_memory_view(m->nbody, m->body_gravcomp)); }
val  body_margin() const { return val(typed_memory_view(m->nbody, m->body_margin)); }
val  body_user() const { return val(typed_memory_view(m->nbody, m->body_user)); }
val  body_plugin() const { return val(typed_memory_view(m->nbody, m->body_plugin)); }
val  body_contype() const { return val(typed_memory_view(m->nbody, m->body_contype)); }
val  body_conaffinity() const { return val(typed_memory_view(m->nbody, m->body_conaffinity)); }
val  jnt_qposadr() const { return val(typed_memory_view(m->njnt, m->jnt_qposadr)); }
val  jnt_dofadr() const { return val(typed_memory_view(m->njnt, m->jnt_dofadr)); }
val  jnt_limited() const { return val(typed_memory_view(m->njnt, m->jnt_limited)); }
val  jnt_actfrclimited() const { return val(typed_memory_view(m->njnt, m->jnt_actfrclimited)); }
val  jnt_actgravcomp() const { return val(typed_memory_view(m->njnt, m->jnt_actgravcomp)); }
val  jnt_solref() const { return val(typed_memory_view(m->njnt, m->jnt_solref)); }
val  jnt_solimp() const { return val(typed_memory_view(m->njnt, m->jnt_solimp)); }
val  jnt_pos() const { return val(typed_memory_view(m->njnt, m->jnt_pos)); }
val  jnt_axis() const { return val(typed_memory_view(m->njnt, m->jnt_axis)); }
val  jnt_stiffness() const { return val(typed_memory_view(m->njnt, m->jnt_stiffness)); }
val  jnt_range() const { return val(typed_memory_view(m->njnt, m->jnt_range)); }
val  jnt_actfrcrange() const { return val(typed_memory_view(m->njnt, m->jnt_actfrcrange)); }
val  jnt_margin() const { return val(typed_memory_view(m->njnt, m->jnt_margin)); }
val  jnt_user() const { return val(typed_memory_view(m->njnt, m->jnt_user)); }
val  dof_bodyid() const { return val(typed_memory_view(m->nv, m->dof_bodyid)); }
val  dof_jntid() const { return val(typed_memory_view(m->nv, m->dof_jntid)); }
val  dof_parentid() const { return val(typed_memory_view(m->nv, m->dof_parentid)); }
val  dof_treeid() const { return val(typed_memory_view(m->nv, m->dof_treeid)); }
val  dof_Madr() const { return val(typed_memory_view(m->nv, m->dof_Madr)); }
val  dof_simplenum() const { return val(typed_memory_view(m->nv, m->dof_simplenum)); }
val  dof_solref() const { return val(typed_memory_view(m->nv, m->dof_solref)); }
val  dof_solimp() const { return val(typed_memory_view(m->nv, m->dof_solimp)); }
val  dof_frictionloss() const { return val(typed_memory_view(m->nv, m->dof_frictionloss)); }
val  dof_armature() const { return val(typed_memory_view(m->nv, m->dof_armature)); }
val  dof_damping() const { return val(typed_memory_view(m->nv, m->dof_damping)); }
val  dof_invweight0() const { return val(typed_memory_view(m->nv, m->dof_invweight0)); }
val  dof_M0() const { return val(typed_memory_view(m->nv, m->dof_M0)); }
val  geom_condim() const { return val(typed_memory_view(m->ngeom, m->geom_condim)); }
val  geom_priority() const { return val(typed_memory_view(m->ngeom, m->geom_priority)); }
val  geom_plugin() const { return val(typed_memory_view(m->ngeom, m->geom_plugin)); }
val  geom_sameframe() const { return val(typed_memory_view(m->ngeom, m->geom_sameframe)); }
val  geom_solmix() const { return val(typed_memory_view(m->ngeom, m->geom_solmix)); }
val  geom_solref() const { return val(typed_memory_view(m->ngeom, m->geom_solref)); }
val  geom_solimp() const { return val(typed_memory_view(m->ngeom, m->geom_solimp)); }
val  geom_pos() const { return val(typed_memory_view(m->ngeom, m->geom_pos)); }
val  geom_quat() const { return val(typed_memory_view(m->ngeom, m->geom_quat)); }
val  geom_friction() const { return val(typed_memory_view(m->ngeom, m->geom_friction)); }
val  geom_margin() const { return val(typed_memory_view(m->ngeom, m->geom_margin)); }
val  geom_gap() const { return val(typed_memory_view(m->ngeom, m->geom_gap)); }
val  geom_fluid() const { return val(typed_memory_view(m->ngeom, m->geom_fluid)); }
val  geom_user() const { return val(typed_memory_view(m->ngeom, m->geom_user)); }
val  site_sameframe() const { return val(typed_memory_view(m->nsite, m->site_sameframe)); }
val  site_pos() const { return val(typed_memory_view(m->nsite, m->site_pos)); }
val  site_quat() const { return val(typed_memory_view(m->nsite, m->site_quat)); }
val  site_user() const { return val(typed_memory_view(m->nsite, m->site_user)); }
val  cam_mode() const { return val(typed_memory_view(m->ncam, m->cam_mode)); }
val  cam_bodyid() const { return val(typed_memory_view(m->ncam, m->cam_bodyid)); }
val  cam_targetbodyid() const { return val(typed_memory_view(m->ncam, m->cam_targetbodyid)); }
val  cam_pos() const { return val(typed_memory_view(m->ncam, m->cam_pos)); }
val  cam_quat() const { return val(typed_memory_view(m->ncam, m->cam_quat)); }
val  cam_poscom0() const { return val(typed_memory_view(m->ncam, m->cam_poscom0)); }
val  cam_pos0() const { return val(typed_memory_view(m->ncam, m->cam_pos0)); }
val  cam_mat0() const { return val(typed_memory_view(m->ncam, m->cam_mat0)); }
val  cam_user() const { return val(typed_memory_view(m->ncam, m->cam_user)); }
val  light_mode() const { return val(typed_memory_view(m->nlight, m->light_mode)); }
val  light_bodyid() const { return val(typed_memory_view(m->nlight, m->light_bodyid)); }
val  light_targetbodyid() const { return val(typed_memory_view(m->nlight, m->light_targetbodyid)); }
val  light_pos() const { return val(typed_memory_view(m->nlight, m->light_pos)); }
val  light_dir() const { return val(typed_memory_view(m->nlight, m->light_dir)); }
val  light_poscom0() const { return val(typed_memory_view(m->nlight, m->light_poscom0)); }
val  light_pos0() const { return val(typed_memory_view(m->nlight, m->light_pos0)); }
val  light_dir0() const { return val(typed_memory_view(m->nlight, m->light_dir0)); }
val  flex_contype() const { return val(typed_memory_view(m->nflex, m->flex_contype)); }
val  flex_conaffinity() const { return val(typed_memory_view(m->nflex, m->flex_conaffinity)); }
val  flex_condim() const { return val(typed_memory_view(m->nflex, m->flex_condim)); }
val  flex_priority() const { return val(typed_memory_view(m->nflex, m->flex_priority)); }
val  flex_solmix() const { return val(typed_memory_view(m->nflex, m->flex_solmix)); }
val  flex_solref() const { return val(typed_memory_view(m->nflex, m->flex_solref)); }
val  flex_solimp() const { return val(typed_memory_view(m->nflex, m->flex_solimp)); }
val  flex_friction() const { return val(typed_memory_view(m->nflex, m->flex_friction)); }
val  flex_margin() const { return val(typed_memory_view(m->nflex, m->flex_margin)); }
val  flex_gap() const { return val(typed_memory_view(m->nflex, m->flex_gap)); }
val  flex_internal() const { return val(typed_memory_view(m->nflex, m->flex_internal)); }
val  flex_selfcollide() const { return val(typed_memory_view(m->nflex, m->flex_selfcollide)); }
val  flex_activelayers() const { return val(typed_memory_view(m->nflex, m->flex_activelayers)); }
val  flex_edgeadr() const { return val(typed_memory_view(m->nflex, m->flex_edgeadr)); }
val  flex_edgenum() const { return val(typed_memory_view(m->nflex, m->flex_edgenum)); }
val  flex_evpairadr() const { return val(typed_memory_view(m->nflex, m->flex_evpairadr)); }
val  flex_evpairnum() const { return val(typed_memory_view(m->nflex, m->flex_evpairnum)); }
val  flex_vertbodyid() const { return val(typed_memory_view(m->nflexvert, m->flex_vertbodyid)); }
val  flex_edge() const { return val(typed_memory_view(m->nflexedge, m->flex_edge)); }
val  flex_evpair() const { return val(typed_memory_view(m->nflexevpair, m->flex_evpair)); }
val  flex_vert() const { return val(typed_memory_view(m->nflexvert, m->flex_vert)); }
val  flex_xvert0() const { return val(typed_memory_view(m->nflexvert, m->flex_xvert0)); }
val  flexedge_length0() const { return val(typed_memory_view(m->nflexedge, m->flexedge_length0)); }
val  flexedge_invweight0() const { return val(typed_memory_view(m->nflexedge, m->flexedge_invweight0)); }
val  flex_edgestiffness() const { return val(typed_memory_view(m->nflex, m->flex_edgestiffness)); }
val  flex_edgedamping() const { return val(typed_memory_view(m->nflex, m->flex_edgedamping)); }
val  flex_edgeequality() const { return val(typed_memory_view(m->nflex, m->flex_edgeequality)); }
val  flex_rigid() const { return val(typed_memory_view(m->nflex, m->flex_rigid)); }
val  flexedge_rigid() const { return val(typed_memory_view(m->nflexedge, m->flexedge_rigid)); }
val  flex_centered() const { return val(typed_memory_view(m->nflex, m->flex_centered)); }
val  flex_texcoord() const { return val(typed_memory_view(m->nflextexcoord, m->flex_texcoord)); }
val  mesh_vertadr() const { return val(typed_memory_view(m->nmesh, m->mesh_vertadr)); }
val  mesh_vertnum() const { return val(typed_memory_view(m->nmesh, m->mesh_vertnum)); }
val  mesh_normaladr() const { return val(typed_memory_view(m->nmesh, m->mesh_normaladr)); }
val  mesh_normalnum() const { return val(typed_memory_view(m->nmesh, m->mesh_normalnum)); }
val  mesh_texcoordnum() const { return val(typed_memory_view(m->nmesh, m->mesh_texcoordnum)); }
val  mesh_faceadr() const { return val(typed_memory_view(m->nmesh, m->mesh_faceadr)); }
val  mesh_facenum() const { return val(typed_memory_view(m->nmesh, m->mesh_facenum)); }
val  mesh_scale() const { return val(typed_memory_view(m->nmesh, m->mesh_scale)); }
val  mesh_pos() const { return val(typed_memory_view(m->nmesh, m->mesh_pos)); }
val  mesh_quat() const { return val(typed_memory_view(m->nmesh, m->mesh_quat)); }
val  mesh_vert() const { return val(typed_memory_view(m->nmeshvert, m->mesh_vert)); }
val  mesh_normal() const { return val(typed_memory_view(m->nmeshnormal, m->mesh_normal)); }
val  mesh_texcoord() const { return val(typed_memory_view(m->nmeshtexcoord, m->mesh_texcoord)); }
val  mesh_face() const { return val(typed_memory_view(m->nmeshface, m->mesh_face)); }
val  mesh_facenormal() const { return val(typed_memory_view(m->nmeshface, m->mesh_facenormal)); }
val  mesh_facetexcoord() const { return val(typed_memory_view(m->nmeshface, m->mesh_facetexcoord)); }
val  mesh_graph() const { return val(typed_memory_view(m->nmeshgraph, m->mesh_graph)); }
val  skin_texcoord() const { return val(typed_memory_view(m->nskintexvert, m->skin_texcoord)); }
val  hfield_size() const { return val(typed_memory_view(m->nhfield, m->hfield_size)); }
val  hfield_nrow() const { return val(typed_memory_view(m->nhfield, m->hfield_nrow)); }
val  hfield_ncol() const { return val(typed_memory_view(m->nhfield, m->hfield_ncol)); }
val  hfield_adr() const { return val(typed_memory_view(m->nhfield, m->hfield_adr)); }
val  hfield_data() const { return val(typed_memory_view(m->nhfielddata, m->hfield_data)); }
val  tex_type() const { return val(typed_memory_view(m->ntex, m->tex_type)); }
val  tex_height() const { return val(typed_memory_view(m->ntex, m->tex_height)); }
val  tex_width() const { return val(typed_memory_view(m->ntex, m->tex_width)); }
val  tex_nchannel() const { return val(typed_memory_view(m->ntex, m->tex_nchannel)); }
val  tex_adr() const { return val(typed_memory_view(m->ntex, m->tex_adr)); }
val  tex_data() const { return val(typed_memory_view(m->ntexdata, m->tex_data)); }
val  pair_dim() const { return val(typed_memory_view(m->npair, m->pair_dim)); }
val  pair_geom1() const { return val(typed_memory_view(m->npair, m->pair_geom1)); }
val  pair_geom2() const { return val(typed_memory_view(m->npair, m->pair_geom2)); }
val  pair_signature() const { return val(typed_memory_view(m->npair, m->pair_signature)); }
val  pair_solref() const { return val(typed_memory_view(m->npair, m->pair_solref)); }
val  pair_solreffriction() const { return val(typed_memory_view(m->npair, m->pair_solreffriction)); }
val  pair_solimp() const { return val(typed_memory_view(m->npair, m->pair_solimp)); }
val  pair_margin() const { return val(typed_memory_view(m->npair, m->pair_margin)); }
val  pair_gap() const { return val(typed_memory_view(m->npair, m->pair_gap)); }
val  pair_friction() const { return val(typed_memory_view(m->npair, m->pair_friction)); }
val  exclude_signature() const { return val(typed_memory_view(m->nexclude, m->exclude_signature)); }
val  eq_active0() const { return val(typed_memory_view(m->neq, m->eq_active0)); }
val  eq_solref() const { return val(typed_memory_view(m->neq, m->eq_solref)); }
val  eq_solimp() const { return val(typed_memory_view(m->neq, m->eq_solimp)); }
val  tendon_adr() const { return val(typed_memory_view(m->ntendon, m->tendon_adr)); }
val  tendon_solref_lim() const { return val(typed_memory_view(m->ntendon, m->tendon_solref_lim)); }
val  tendon_solimp_lim() const { return val(typed_memory_view(m->ntendon, m->tendon_solimp_lim)); }
val  tendon_solref_fri() const { return val(typed_memory_view(m->ntendon, m->tendon_solref_fri)); }
val  tendon_solimp_fri() const { return val(typed_memory_view(m->ntendon, m->tendon_solimp_fri)); }
val  tendon_margin() const { return val(typed_memory_view(m->ntendon, m->tendon_margin)); }
val  tendon_length0() const { return val(typed_memory_view(m->ntendon, m->tendon_length0)); }
val  tendon_invweight0() const { return val(typed_memory_view(m->ntendon, m->tendon_invweight0)); }
val  tendon_user() const { return val(typed_memory_view(m->ntendon, m->tendon_user)); }
val  wrap_type() const { return val(typed_memory_view(m->nwrap, m->wrap_type)); }
val  wrap_objid() const { return val(typed_memory_view(m->nwrap, m->wrap_objid)); }
val  wrap_prm() const { return val(typed_memory_view(m->nwrap, m->wrap_prm)); }
val  actuator_gaintype() const { return val(typed_memory_view(m->nu, m->actuator_gaintype)); }
val  actuator_biastype() const { return val(typed_memory_view(m->nu, m->actuator_biastype)); }
val  actuator_forcelimited() const { return val(typed_memory_view(m->nu, m->actuator_forcelimited)); }
val  actuator_dynprm() const { return val(typed_memory_view(m->nu, m->actuator_dynprm)); }
val  actuator_gainprm() const { return val(typed_memory_view(m->nu, m->actuator_gainprm)); }
val  actuator_biasprm() const { return val(typed_memory_view(m->nu, m->actuator_biasprm)); }
val  actuator_actearly() const { return val(typed_memory_view(m->nu, m->actuator_actearly)); }
val  actuator_forcerange() const { return val(typed_memory_view(m->nu, m->actuator_forcerange)); }
val  actuator_gear() const { return val(typed_memory_view(m->nu, m->actuator_gear)); }
val  actuator_acc0() const { return val(typed_memory_view(m->nu, m->actuator_acc0)); }
val  actuator_length0() const { return val(typed_memory_view(m->nu, m->actuator_length0)); }
val  actuator_lengthrange() const { return val(typed_memory_view(m->nu, m->actuator_lengthrange)); }
val  actuator_user() const { return val(typed_memory_view(m->nu, m->actuator_user)); }
val  actuator_plugin() const { return val(typed_memory_view(m->nu, m->actuator_plugin)); }
val  sensor_datatype() const { return val(typed_memory_view(m->nsensor, m->sensor_datatype)); }
val  sensor_needstage() const { return val(typed_memory_view(m->nsensor, m->sensor_needstage)); }
val  sensor_objtype() const { return val(typed_memory_view(m->nsensor, m->sensor_objtype)); }
val  sensor_reftype() const { return val(typed_memory_view(m->nsensor, m->sensor_reftype)); }
val  sensor_refid() const { return val(typed_memory_view(m->nsensor, m->sensor_refid)); }
val  sensor_dim() const { return val(typed_memory_view(m->nsensor, m->sensor_dim)); }
val  sensor_cutoff() const { return val(typed_memory_view(m->nsensor, m->sensor_cutoff)); }
val  sensor_noise() const { return val(typed_memory_view(m->nsensor, m->sensor_noise)); }
val  sensor_user() const { return val(typed_memory_view(m->nsensor, m->sensor_user)); }
val  sensor_plugin() const { return val(typed_memory_view(m->nsensor, m->sensor_plugin)); }
val  plugin() const { return val(typed_memory_view(m->nplugin, m->plugin)); }
val  plugin_stateadr() const { return val(typed_memory_view(m->nplugin, m->plugin_stateadr)); }
val  plugin_statenum() const { return val(typed_memory_view(m->nplugin, m->plugin_statenum)); }
val  plugin_attr() const { return val(typed_memory_view(m->npluginattr, m->plugin_attr)); }
val  plugin_attradr() const { return val(typed_memory_view(m->nplugin, m->plugin_attradr)); }
val  numeric_adr() const { return val(typed_memory_view(m->nnumeric, m->numeric_adr)); }
val  numeric_size() const { return val(typed_memory_view(m->nnumeric, m->numeric_size)); }
val  numeric_data() const { return val(typed_memory_view(m->nnumericdata, m->numeric_data)); }
val  text_adr() const { return val(typed_memory_view(m->ntext, m->text_adr)); }
val  text_size() const { return val(typed_memory_view(m->ntext, m->text_size)); }
val  text_data() const { return val(typed_memory_view(m->ntextdata, m->text_data)); }
val  tuple_adr() const { return val(typed_memory_view(m->ntuple, m->tuple_adr)); }
val  tuple_size() const { return val(typed_memory_view(m->ntuple, m->tuple_size)); }
val  tuple_objtype() const { return val(typed_memory_view(m->ntupledata, m->tuple_objtype)); }
val  tuple_objid() const { return val(typed_memory_view(m->ntupledata, m->tuple_objid)); }
val  tuple_objprm() const { return val(typed_memory_view(m->ntupledata, m->tuple_objprm)); }
val  key_time() const { return val(typed_memory_view(m->nkey, m->key_time)); }
val  key_qpos() const { return val(typed_memory_view(m->nkey, m->key_qpos)); }
val  key_qvel() const { return val(typed_memory_view(m->nkey, m->key_qvel)); }
val  key_act() const { return val(typed_memory_view(m->nkey, m->key_act)); }
val  key_mpos() const { return val(typed_memory_view(m->nkey, m->key_mpos)); }
val  key_mquat() const { return val(typed_memory_view(m->nkey, m->key_mquat)); }
val  key_ctrl() const { return val(typed_memory_view(m->nkey, m->key_ctrl)); }
val  name_flexadr() const { return val(typed_memory_view(m->nflex, m->name_flexadr)); }
val  name_meshadr() const { return val(typed_memory_view(m->nmesh, m->name_meshadr)); }
val  name_skinadr() const { return val(typed_memory_view(m->nskin, m->name_skinadr)); }
val  name_hfieldadr() const { return val(typed_memory_view(m->nhfield, m->name_hfieldadr)); }
val  name_texadr() const { return val(typed_memory_view(m->ntex, m->name_texadr)); }
val  name_matadr() const { return val(typed_memory_view(m->nmat, m->name_matadr)); }
val  name_pairadr() const { return val(typed_memory_view(m->npair, m->name_pairadr)); }
val  name_excludeadr() const { return val(typed_memory_view(m->nexclude, m->name_excludeadr)); }
val  name_sensoradr() const { return val(typed_memory_view(m->nsensor, m->name_sensoradr)); }
val  name_numericadr() const { return val(typed_memory_view(m->nnumeric, m->name_numericadr)); }
val  name_textadr() const { return val(typed_memory_view(m->ntext, m->name_textadr)); }
val  name_tupleadr() const { return val(typed_memory_view(m->ntuple, m->name_tupleadr)); }
val  name_keyadr() const { return val(typed_memory_view(m->nkey, m->name_keyadr)); }
val  name_pluginadr() const { return val(typed_memory_view(m->nplugin, m->name_pluginadr)); }
val  names_map() const { return val(typed_memory_view(m->nnames_map, m->names_map)); }

private:
  mjModel *m;
};

class State {
public:
  State(Model m)  { d = mj_makeData(m.ptr()); }
  mjData *ptr  () { return d; }
  mjData getVal() { return *d; }
  void free    () { return mju_free(d); }

private:
  mjData *d;
};

class Simulation {
public:
  Simulation(Model *m, State *s) {
    _model = m;
    _state = s;
  }

  State *state() { return _state; }
  Model *model() { return _model; }
  void    free() { mju_free(_state); mju_free(_model); }

  void applyForce(
    mjtNum fx, mjtNum fy, mjtNum fz, 
    mjtNum tx, mjtNum ty, mjtNum tz,  
    mjtNum px, mjtNum py, mjtNum pz, int body) {
    mjtNum force [3] = {fx, fy, fz};
    mjtNum torque[3] = {tx, ty, tz};
    mjtNum point [3] = {px, py, pz};
    mj_applyFT(_model->ptr(), _state->ptr(), 
               force, torque, point, body, 
               _state->ptr()->qfrc_applied);
  }

  // copied from the source of mjv_applyPerturbPose
  // sets perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
  //  d->qpos written only if flg_paused and subtree root for selected body has free joint
  void applyPose(int bodyID,
                 mjtNum refPosX,  mjtNum refPosY,  mjtNum refPosZ,
                 mjtNum refQuat1, mjtNum refQuat2, mjtNum refQuat3, mjtNum refQuat4,
                 int flg_paused) {
    int rootid = 0, sel = bodyID;//pert->select;
    mjtNum pos1[3], quat1[4], pos2[3], quat2[4], refpos[3], refquat[4];
    mjtNum *Rpos, *Rquat, *Cpos, *Cquat;
    mjtNum inrefpos [3] = { refPosX , refPosY , refPosZ };
    mjtNum inrefquat[4] = { refQuat1, refQuat2, refQuat3, refQuat4 };
    mjModel *m = _model->ptr();
    mjData  *d = _state->ptr();

    // exit if nothing to do
    //if (sel<=0 || sel>=m->nbody || !(pert->active | pert->active2)) { return; }

    // get rootid above selected body
    rootid = m->body_rootid[sel];

    // transform refpos,refquat from I-frame to X-frame of body[sel]
    mju_negPose(pos1, quat1, m->body_ipos+3*sel, m->body_iquat+4*sel);
    mju_mulPose(refpos, refquat, inrefpos, inrefquat, pos1, quat1);

    // mocap body
    if (m->body_mocapid[sel]>=0) {
      // copy ref pose into mocap pose
      mju_copy3(d->mocap_pos + 3*m->body_mocapid[sel], refpos);
      mju_copy4(d->mocap_quat + 4*m->body_mocapid[sel], refquat);
    }

    // floating body, paused
    else if (flg_paused && m->body_jntnum[sel]==1 &&
            m->jnt_type[m->body_jntadr[sel]]==mjJNT_FREE) {
      // copy ref pose into qpos
      mju_copy3(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]], refpos);
      mju_copy4(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]] + 3, refquat);
    }

    // child of floating body, paused
    else if (flg_paused && m->body_jntnum[rootid]==1 &&
            m->jnt_type[m->body_jntadr[rootid]]==mjJNT_FREE) {
      // get pointers to root
      Rpos = d->qpos + m->jnt_qposadr[m->body_jntadr[rootid]];
      Rquat = Rpos + 3;

      // get pointers to child
      Cpos = d->xpos + 3*sel;
      Cquat = d->xquat + 4*sel;

      // set root <- ref*neg(child)*root
      mju_negPose(pos1, quat1, Cpos, Cquat);                      // neg(child)
      mju_mulPose(pos2, quat2, pos1, quat1, Rpos, Rquat);         // neg(child)*root
      mju_mulPose(Rpos, Rquat, refpos, refquat, pos2, quat2);     // ref*neg(child)*root
    }
  }

  // MJDATA_DEFINITIONS
val  qpos() const { return val(typed_memory_view(_model->ptr()->nq * 1, _state->ptr()->qpos)); }
val  qvel() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qvel)); }
val  qacc_warmstart() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qacc_warmstart)); }
val  plugin_state() const { return val(typed_memory_view(_model->ptr()->npluginstate * 1, _state->ptr()->plugin_state)); }
val  qfrc_applied() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_applied)); }
val  mocap_pos() const { return val(typed_memory_view(_model->ptr()->nmocap * 3, _state->ptr()->mocap_pos)); }
val  mocap_quat() const { return val(typed_memory_view(_model->ptr()->nmocap * 4, _state->ptr()->mocap_quat)); }
val  qacc() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qacc)); }
val  act_dot() const { return val(typed_memory_view(_model->ptr()->na * 1, _state->ptr()->act_dot)); }
val  userdata() const { return val(typed_memory_view(_model->ptr()->nuserdata * 1, _state->ptr()->userdata)); }
val  plugin() const { return val(typed_memory_view(_model->ptr()->nplugin * 1, _state->ptr()->plugin)); }
val  plugin_data() const { return val(typed_memory_view(_model->ptr()->nplugin * 1, _state->ptr()->plugin_data)); }
val  cdof() const { return val(typed_memory_view(_model->ptr()->nv * 6, _state->ptr()->cdof)); }
val  cinert() const { return val(typed_memory_view(_model->ptr()->nbody * 10, _state->ptr()->cinert)); }
val  flexelem_aabb() const { return val(typed_memory_view(_model->ptr()->nflexelem * 6, _state->ptr()->flexelem_aabb)); }
val  flexedge_J_rownnz() const { return val(typed_memory_view(_model->ptr()->nflexedge * 1, _state->ptr()->flexedge_J_rownnz)); }
val  flexedge_J_rowadr() const { return val(typed_memory_view(_model->ptr()->nflexedge * 1, _state->ptr()->flexedge_J_rowadr)); }
val  flexedge_length() const { return val(typed_memory_view(_model->ptr()->nflexedge * 1, _state->ptr()->flexedge_length)); }
val  ten_J_rownnz() const { return val(typed_memory_view(_model->ptr()->ntendon * 1, _state->ptr()->ten_J_rownnz)); }
val  ten_J_rowadr() const { return val(typed_memory_view(_model->ptr()->ntendon * 1, _state->ptr()->ten_J_rowadr)); }
val  actuator_length() const { return val(typed_memory_view(_model->ptr()->nu * 1, _state->ptr()->actuator_length)); }
val  crb() const { return val(typed_memory_view(_model->ptr()->nbody * 10, _state->ptr()->crb)); }
val  qM() const { return val(typed_memory_view(_model->ptr()->nM * 1, _state->ptr()->qM)); }
val  qLD() const { return val(typed_memory_view(_model->ptr()->nM * 1, _state->ptr()->qLD)); }
val  qLDiagInv() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qLDiagInv)); }
val  qLDiagSqrtInv() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qLDiagSqrtInv)); }
val  flexedge_velocity() const { return val(typed_memory_view(_model->ptr()->nflexedge * 1, _state->ptr()->flexedge_velocity)); }
val  ten_velocity() const { return val(typed_memory_view(_model->ptr()->ntendon * 1, _state->ptr()->ten_velocity)); }
val  actuator_velocity() const { return val(typed_memory_view(_model->ptr()->nu * 1, _state->ptr()->actuator_velocity)); }
val  cvel() const { return val(typed_memory_view(_model->ptr()->nbody * 6, _state->ptr()->cvel)); }
val  cdof_dot() const { return val(typed_memory_view(_model->ptr()->nv * 6, _state->ptr()->cdof_dot)); }
val  qfrc_bias() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_bias)); }
val  qfrc_spring() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_spring)); }
val  qfrc_damper() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_damper)); }
val  qfrc_gravcomp() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_gravcomp)); }
val  qfrc_fluid() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_fluid)); }
val  qfrc_passive() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_passive)); }
val  subtree_linvel() const { return val(typed_memory_view(_model->ptr()->nbody * 3, _state->ptr()->subtree_linvel)); }
val  subtree_angmom() const { return val(typed_memory_view(_model->ptr()->nbody * 3, _state->ptr()->subtree_angmom)); }
val  qH() const { return val(typed_memory_view(_model->ptr()->nM * 1, _state->ptr()->qH)); }
val  qHDiagInv() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qHDiagInv)); }
val  B_rownnz() const { return val(typed_memory_view(_model->ptr()->nbody * 1, _state->ptr()->B_rownnz)); }
val  B_rowadr() const { return val(typed_memory_view(_model->ptr()->nbody * 1, _state->ptr()->B_rowadr)); }
val  B_colind() const { return val(typed_memory_view(_model->ptr()->nB * 1, _state->ptr()->B_colind)); }
val  C_rownnz() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->C_rownnz)); }
val  C_rowadr() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->C_rowadr)); }
val  C_colind() const { return val(typed_memory_view(_model->ptr()->nC * 1, _state->ptr()->C_colind)); }
val  mapM2C() const { return val(typed_memory_view(_model->ptr()->nC * 1, _state->ptr()->mapM2C)); }
val  D_rownnz() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->D_rownnz)); }
val  D_rowadr() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->D_rowadr)); }
val  D_colind() const { return val(typed_memory_view(_model->ptr()->nD * 1, _state->ptr()->D_colind)); }
val  mapM2D() const { return val(typed_memory_view(_model->ptr()->nD * 1, _state->ptr()->mapM2D)); }
val  mapD2M() const { return val(typed_memory_view(_model->ptr()->nM * 1, _state->ptr()->mapD2M)); }
val  qDeriv() const { return val(typed_memory_view(_model->ptr()->nD * 1, _state->ptr()->qDeriv)); }
val  qLU() const { return val(typed_memory_view(_model->ptr()->nD * 1, _state->ptr()->qLU)); }
val  actuator_force() const { return val(typed_memory_view(_model->ptr()->nu * 1, _state->ptr()->actuator_force)); }
val  qfrc_actuator() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_actuator)); }
val  qfrc_smooth() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_smooth)); }
val  qacc_smooth() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qacc_smooth)); }
val  qfrc_constraint() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_constraint)); }
val  qfrc_inverse() const { return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_inverse)); }
val  cacc() const { return val(typed_memory_view(_model->ptr()->nbody * 6, _state->ptr()->cacc)); }
val  cfrc_int() const { return val(typed_memory_view(_model->ptr()->nbody * 6, _state->ptr()->cfrc_int)); }
val  cfrc_ext() const { return val(typed_memory_view(_model->ptr()->nbody * 6, _state->ptr()->cfrc_ext)); }


private:
  Model *_model;
  State *_state;
};

// main function
int main(int argc, char **argv) {
  std::printf("MuJoCo version: %d\n\n", mj_version());
  return 0;
}

EMSCRIPTEN_BINDINGS(mujoco_wasm) {

  // MODEL_ENUMS


  class_<Model>("Model")
      .constructor<>(&Model::load_from_xml)
      .class_function("load_from_xml", &Model::load_from_xml)
      .class_function("load_from_mjb", &Model::load_from_mjb)
      .function("ptr", &Model::ptr, allow_raw_pointers())
      .function("free"            , &Model::free        )
      .function("getVal"          , &Model::getVal      )
      .function("getOptions"      , &Model::getOptions  )
      // MJMODEL_BINDINGS
.property("qpos0", &Model::qpos0)
.property("qpos_spring", &Model::qpos_spring)
.property("body_treeid", &Model::body_treeid)
.property("body_simple", &Model::body_simple)
.property("body_sameframe", &Model::body_sameframe)
.property("body_pos", &Model::body_pos)
.property("body_quat", &Model::body_quat)
.property("body_ipos", &Model::body_ipos)
.property("body_subtreemass", &Model::body_subtreemass)
.property("body_invweight0", &Model::body_invweight0)
.property("body_gravcomp", &Model::body_gravcomp)
.property("body_margin", &Model::body_margin)
.property("body_user", &Model::body_user)
.property("body_plugin", &Model::body_plugin)
.property("body_contype", &Model::body_contype)
.property("body_conaffinity", &Model::body_conaffinity)
.property("jnt_qposadr", &Model::jnt_qposadr)
.property("jnt_dofadr", &Model::jnt_dofadr)
.property("jnt_limited", &Model::jnt_limited)
.property("jnt_actfrclimited", &Model::jnt_actfrclimited)
.property("jnt_actgravcomp", &Model::jnt_actgravcomp)
.property("jnt_solref", &Model::jnt_solref)
.property("jnt_solimp", &Model::jnt_solimp)
.property("jnt_pos", &Model::jnt_pos)
.property("jnt_axis", &Model::jnt_axis)
.property("jnt_stiffness", &Model::jnt_stiffness)
.property("jnt_range", &Model::jnt_range)
.property("jnt_actfrcrange", &Model::jnt_actfrcrange)
.property("jnt_margin", &Model::jnt_margin)
.property("jnt_user", &Model::jnt_user)
.property("dof_bodyid", &Model::dof_bodyid)
.property("dof_jntid", &Model::dof_jntid)
.property("dof_parentid", &Model::dof_parentid)
.property("dof_treeid", &Model::dof_treeid)
.property("dof_Madr", &Model::dof_Madr)
.property("dof_simplenum", &Model::dof_simplenum)
.property("dof_solref", &Model::dof_solref)
.property("dof_solimp", &Model::dof_solimp)
.property("dof_frictionloss", &Model::dof_frictionloss)
.property("dof_armature", &Model::dof_armature)
.property("dof_damping", &Model::dof_damping)
.property("dof_invweight0", &Model::dof_invweight0)
.property("dof_M0", &Model::dof_M0)
.property("geom_condim", &Model::geom_condim)
.property("geom_priority", &Model::geom_priority)
.property("geom_plugin", &Model::geom_plugin)
.property("geom_sameframe", &Model::geom_sameframe)
.property("geom_solmix", &Model::geom_solmix)
.property("geom_solref", &Model::geom_solref)
.property("geom_solimp", &Model::geom_solimp)
.property("geom_pos", &Model::geom_pos)
.property("geom_quat", &Model::geom_quat)
.property("geom_friction", &Model::geom_friction)
.property("geom_margin", &Model::geom_margin)
.property("geom_gap", &Model::geom_gap)
.property("geom_fluid", &Model::geom_fluid)
.property("geom_user", &Model::geom_user)
.property("site_sameframe", &Model::site_sameframe)
.property("site_pos", &Model::site_pos)
.property("site_quat", &Model::site_quat)
.property("site_user", &Model::site_user)
.property("cam_mode", &Model::cam_mode)
.property("cam_bodyid", &Model::cam_bodyid)
.property("cam_targetbodyid", &Model::cam_targetbodyid)
.property("cam_pos", &Model::cam_pos)
.property("cam_quat", &Model::cam_quat)
.property("cam_poscom0", &Model::cam_poscom0)
.property("cam_pos0", &Model::cam_pos0)
.property("cam_mat0", &Model::cam_mat0)
.property("cam_user", &Model::cam_user)
.property("light_mode", &Model::light_mode)
.property("light_bodyid", &Model::light_bodyid)
.property("light_targetbodyid", &Model::light_targetbodyid)
.property("light_pos", &Model::light_pos)
.property("light_dir", &Model::light_dir)
.property("light_poscom0", &Model::light_poscom0)
.property("light_pos0", &Model::light_pos0)
.property("light_dir0", &Model::light_dir0)
.property("flex_contype", &Model::flex_contype)
.property("flex_conaffinity", &Model::flex_conaffinity)
.property("flex_condim", &Model::flex_condim)
.property("flex_priority", &Model::flex_priority)
.property("flex_solmix", &Model::flex_solmix)
.property("flex_solref", &Model::flex_solref)
.property("flex_solimp", &Model::flex_solimp)
.property("flex_friction", &Model::flex_friction)
.property("flex_margin", &Model::flex_margin)
.property("flex_gap", &Model::flex_gap)
.property("flex_internal", &Model::flex_internal)
.property("flex_selfcollide", &Model::flex_selfcollide)
.property("flex_activelayers", &Model::flex_activelayers)
.property("flex_edgeadr", &Model::flex_edgeadr)
.property("flex_edgenum", &Model::flex_edgenum)
.property("flex_evpairadr", &Model::flex_evpairadr)
.property("flex_evpairnum", &Model::flex_evpairnum)
.property("flex_vertbodyid", &Model::flex_vertbodyid)
.property("flex_edge", &Model::flex_edge)
.property("flex_evpair", &Model::flex_evpair)
.property("flex_vert", &Model::flex_vert)
.property("flex_xvert0", &Model::flex_xvert0)
.property("flexedge_length0", &Model::flexedge_length0)
.property("flexedge_invweight0", &Model::flexedge_invweight0)
.property("flex_edgestiffness", &Model::flex_edgestiffness)
.property("flex_edgedamping", &Model::flex_edgedamping)
.property("flex_edgeequality", &Model::flex_edgeequality)
.property("flex_rigid", &Model::flex_rigid)
.property("flexedge_rigid", &Model::flexedge_rigid)
.property("flex_centered", &Model::flex_centered)
.property("flex_texcoord", &Model::flex_texcoord)
.property("mesh_vertadr", &Model::mesh_vertadr)
.property("mesh_vertnum", &Model::mesh_vertnum)
.property("mesh_normaladr", &Model::mesh_normaladr)
.property("mesh_normalnum", &Model::mesh_normalnum)
.property("mesh_texcoordnum", &Model::mesh_texcoordnum)
.property("mesh_faceadr", &Model::mesh_faceadr)
.property("mesh_facenum", &Model::mesh_facenum)
.property("mesh_scale", &Model::mesh_scale)
.property("mesh_pos", &Model::mesh_pos)
.property("mesh_quat", &Model::mesh_quat)
.property("mesh_vert", &Model::mesh_vert)
.property("mesh_normal", &Model::mesh_normal)
.property("mesh_texcoord", &Model::mesh_texcoord)
.property("mesh_face", &Model::mesh_face)
.property("mesh_facenormal", &Model::mesh_facenormal)
.property("mesh_facetexcoord", &Model::mesh_facetexcoord)
.property("mesh_graph", &Model::mesh_graph)
.property("skin_texcoord", &Model::skin_texcoord)
.property("hfield_size", &Model::hfield_size)
.property("hfield_nrow", &Model::hfield_nrow)
.property("hfield_ncol", &Model::hfield_ncol)
.property("hfield_adr", &Model::hfield_adr)
.property("hfield_data", &Model::hfield_data)
.property("tex_type", &Model::tex_type)
.property("tex_height", &Model::tex_height)
.property("tex_width", &Model::tex_width)
.property("tex_nchannel", &Model::tex_nchannel)
.property("tex_adr", &Model::tex_adr)
.property("tex_data", &Model::tex_data)
.property("pair_dim", &Model::pair_dim)
.property("pair_geom1", &Model::pair_geom1)
.property("pair_geom2", &Model::pair_geom2)
.property("pair_signature", &Model::pair_signature)
.property("pair_solref", &Model::pair_solref)
.property("pair_solreffriction", &Model::pair_solreffriction)
.property("pair_solimp", &Model::pair_solimp)
.property("pair_margin", &Model::pair_margin)
.property("pair_gap", &Model::pair_gap)
.property("pair_friction", &Model::pair_friction)
.property("exclude_signature", &Model::exclude_signature)
.property("eq_active0", &Model::eq_active0)
.property("eq_solref", &Model::eq_solref)
.property("eq_solimp", &Model::eq_solimp)
.property("tendon_adr", &Model::tendon_adr)
.property("tendon_solref_lim", &Model::tendon_solref_lim)
.property("tendon_solimp_lim", &Model::tendon_solimp_lim)
.property("tendon_solref_fri", &Model::tendon_solref_fri)
.property("tendon_solimp_fri", &Model::tendon_solimp_fri)
.property("tendon_margin", &Model::tendon_margin)
.property("tendon_length0", &Model::tendon_length0)
.property("tendon_invweight0", &Model::tendon_invweight0)
.property("tendon_user", &Model::tendon_user)
.property("wrap_type", &Model::wrap_type)
.property("wrap_objid", &Model::wrap_objid)
.property("wrap_prm", &Model::wrap_prm)
.property("actuator_gaintype", &Model::actuator_gaintype)
.property("actuator_biastype", &Model::actuator_biastype)
.property("actuator_forcelimited", &Model::actuator_forcelimited)
.property("actuator_dynprm", &Model::actuator_dynprm)
.property("actuator_gainprm", &Model::actuator_gainprm)
.property("actuator_biasprm", &Model::actuator_biasprm)
.property("actuator_actearly", &Model::actuator_actearly)
.property("actuator_forcerange", &Model::actuator_forcerange)
.property("actuator_gear", &Model::actuator_gear)
.property("actuator_acc0", &Model::actuator_acc0)
.property("actuator_length0", &Model::actuator_length0)
.property("actuator_lengthrange", &Model::actuator_lengthrange)
.property("actuator_user", &Model::actuator_user)
.property("actuator_plugin", &Model::actuator_plugin)
.property("sensor_datatype", &Model::sensor_datatype)
.property("sensor_needstage", &Model::sensor_needstage)
.property("sensor_objtype", &Model::sensor_objtype)
.property("sensor_reftype", &Model::sensor_reftype)
.property("sensor_refid", &Model::sensor_refid)
.property("sensor_dim", &Model::sensor_dim)
.property("sensor_cutoff", &Model::sensor_cutoff)
.property("sensor_noise", &Model::sensor_noise)
.property("sensor_user", &Model::sensor_user)
.property("sensor_plugin", &Model::sensor_plugin)
.property("plugin", &Model::plugin)
.property("plugin_stateadr", &Model::plugin_stateadr)
.property("plugin_statenum", &Model::plugin_statenum)
.property("plugin_attr", &Model::plugin_attr)
.property("plugin_attradr", &Model::plugin_attradr)
.property("numeric_adr", &Model::numeric_adr)
.property("numeric_size", &Model::numeric_size)
.property("numeric_data", &Model::numeric_data)
.property("text_adr", &Model::text_adr)
.property("text_size", &Model::text_size)
.property("text_data", &Model::text_data)
.property("tuple_adr", &Model::tuple_adr)
.property("tuple_size", &Model::tuple_size)
.property("tuple_objtype", &Model::tuple_objtype)
.property("tuple_objid", &Model::tuple_objid)
.property("tuple_objprm", &Model::tuple_objprm)
.property("key_time", &Model::key_time)
.property("key_qpos", &Model::key_qpos)
.property("key_qvel", &Model::key_qvel)
.property("key_act", &Model::key_act)
.property("key_mpos", &Model::key_mpos)
.property("key_mquat", &Model::key_mquat)
.property("key_ctrl", &Model::key_ctrl)
.property("name_flexadr", &Model::name_flexadr)
.property("name_meshadr", &Model::name_meshadr)
.property("name_skinadr", &Model::name_skinadr)
.property("name_hfieldadr", &Model::name_hfieldadr)
.property("name_texadr", &Model::name_texadr)
.property("name_matadr", &Model::name_matadr)
.property("name_pairadr", &Model::name_pairadr)
.property("name_excludeadr", &Model::name_excludeadr)
.property("name_sensoradr", &Model::name_sensoradr)
.property("name_numericadr", &Model::name_numericadr)
.property("name_textadr", &Model::name_textadr)
.property("name_tupleadr", &Model::name_tupleadr)
.property("name_keyadr", &Model::name_keyadr)
.property("name_pluginadr", &Model::name_pluginadr)
.property("names_map", &Model::names_map)
;

  class_<State>("State")
      .constructor<Model>()
      .function("ptr"   , &State::ptr, allow_raw_pointers())
      .function("free"  , &State::free  )
      .function("getVal", &State::getVal);

  class_<Simulation>("Simulation")
      .constructor<Model *, State *>()
      .function("state"     , &Simulation::state, allow_raw_pointers())
      .function("model"     , &Simulation::model, allow_raw_pointers())
      .function("free"      , &Simulation::free      )
      .function("applyForce", &Simulation::applyForce)
      .function("applyPose" , &Simulation::applyPose )
      // MJDATA_BINDINGS
.property("qpos", &Simulation::qpos)
.property("qvel", &Simulation::qvel)
.property("qacc_warmstart", &Simulation::qacc_warmstart)
.property("plugin_state", &Simulation::plugin_state)
.property("qfrc_applied", &Simulation::qfrc_applied)
.property("mocap_pos", &Simulation::mocap_pos)
.property("mocap_quat", &Simulation::mocap_quat)
.property("qacc", &Simulation::qacc)
.property("act_dot", &Simulation::act_dot)
.property("userdata", &Simulation::userdata)
.property("plugin", &Simulation::plugin)
.property("plugin_data", &Simulation::plugin_data)
.property("cdof", &Simulation::cdof)
.property("cinert", &Simulation::cinert)
.property("flexelem_aabb", &Simulation::flexelem_aabb)
.property("flexedge_J_rownnz", &Simulation::flexedge_J_rownnz)
.property("flexedge_J_rowadr", &Simulation::flexedge_J_rowadr)
.property("flexedge_length", &Simulation::flexedge_length)
.property("ten_J_rownnz", &Simulation::ten_J_rownnz)
.property("ten_J_rowadr", &Simulation::ten_J_rowadr)
.property("actuator_length", &Simulation::actuator_length)
.property("crb", &Simulation::crb)
.property("qM", &Simulation::qM)
.property("qLD", &Simulation::qLD)
.property("qLDiagInv", &Simulation::qLDiagInv)
.property("qLDiagSqrtInv", &Simulation::qLDiagSqrtInv)
.property("flexedge_velocity", &Simulation::flexedge_velocity)
.property("ten_velocity", &Simulation::ten_velocity)
.property("actuator_velocity", &Simulation::actuator_velocity)
.property("cvel", &Simulation::cvel)
.property("cdof_dot", &Simulation::cdof_dot)
.property("qfrc_bias", &Simulation::qfrc_bias)
.property("qfrc_spring", &Simulation::qfrc_spring)
.property("qfrc_damper", &Simulation::qfrc_damper)
.property("qfrc_gravcomp", &Simulation::qfrc_gravcomp)
.property("qfrc_fluid", &Simulation::qfrc_fluid)
.property("qfrc_passive", &Simulation::qfrc_passive)
.property("subtree_linvel", &Simulation::subtree_linvel)
.property("subtree_angmom", &Simulation::subtree_angmom)
.property("qH", &Simulation::qH)
.property("qHDiagInv", &Simulation::qHDiagInv)
.property("B_rownnz", &Simulation::B_rownnz)
.property("B_rowadr", &Simulation::B_rowadr)
.property("B_colind", &Simulation::B_colind)
.property("C_rownnz", &Simulation::C_rownnz)
.property("C_rowadr", &Simulation::C_rowadr)
.property("C_colind", &Simulation::C_colind)
.property("mapM2C", &Simulation::mapM2C)
.property("D_rownnz", &Simulation::D_rownnz)
.property("D_rowadr", &Simulation::D_rowadr)
.property("D_colind", &Simulation::D_colind)
.property("mapM2D", &Simulation::mapM2D)
.property("mapD2M", &Simulation::mapD2M)
.property("qDeriv", &Simulation::qDeriv)
.property("qLU", &Simulation::qLU)
.property("actuator_force", &Simulation::actuator_force)
.property("qfrc_actuator", &Simulation::qfrc_actuator)
.property("qfrc_smooth", &Simulation::qfrc_smooth)
.property("qacc_smooth", &Simulation::qacc_smooth)
.property("qfrc_constraint", &Simulation::qfrc_constraint)
.property("qfrc_inverse", &Simulation::qfrc_inverse)
.property("cacc", &Simulation::cacc)
.property("cfrc_int", &Simulation::cfrc_int)
.property("cfrc_ext", &Simulation::cfrc_ext)
      ;

  value_object<mjModel>("mjModel")
      .field("ngeom"      , &mjModel::ngeom)
      .field("nq"         , &mjModel::nq)
      .field("na"         , &mjModel::na)
      .field("nv"         , &mjModel::nv)
      .field("nu"         , &mjModel::nu)
      .field("nbody"      , &mjModel::nbody)
      .field("nsensordata", &mjModel::nsensordata)
      //.field("body_rootid", &mjModel::body_rootid, allow_raw_pointers())
      .field("nmesh"      , &mjModel::nmesh)
      .field("nmeshvert"  , &mjModel::nmeshvert)
      .field("nmeshface"  , &mjModel::nmeshface);

  value_object<mjvPerturb>("mjvPerturb")
      .field("select"    , &mjvPerturb::select)     // selected body id; non-positive: none
      .field("skinselect", &mjvPerturb::skinselect) // selected skin id; negative: none
      .field("active"    , &mjvPerturb::active)     // perturbation bitmask (mjtPertBit)
      .field("active2"   , &mjvPerturb::active2)    // secondary perturbation bitmask (mjtPertBit)
      .field("refpos"    , &mjvPerturb::refpos)     // desired position for selected object
      .field("refquat"   , &mjvPerturb::refquat)    // desired orientation for selected object
      .field("localpos"  , &mjvPerturb::localpos)   // selection point in object coordinates
      .field("scale"     , &mjvPerturb::scale)      // relative mouse motion-to-space scaling (set by initPerturb)
      ;

  value_object<mjContact>("mjContact")
      .field("dist"         , &mjContact::dist)             // distance between nearest points; neg: penetration
      .field("pos"          , &mjContact::pos)              // position of contact point: midpoint between geoms
      .field("frame"        , &mjContact::frame)            // normal is in [0-2]
      .field("includemargin", &mjContact::includemargin)    // include if dist<includemargin=margin-gap
      .field("friction"     , &mjContact::friction)         // tangent1, 2, spin, roll1, 2
      .field("solref"       , &mjContact::solref)           // constraint solver reference
      .field("solimp"       , &mjContact::solimp)           // constraint solver impedance
      .field("mu"           , &mjContact::mu)               // friction of regularized cone, set by mj_makeConstraint
      .field("H"            , &mjContact::H)                // cone Hessian, set by mj_updateConstraint
      .field("dim"          , &mjContact::H)                // contact space dimensionality: 1, 3, 4 or 6
      .field("geom1"        , &mjContact::H)                // id of geom 1
      .field("geom2"        , &mjContact::H)                // id of geom 2
      .field("exclude"      , &mjContact::exclude)          // 0: include, 1: in gap, 2: fused, 3: equality, 4: no dofs
      .field("efc_address"  , &mjContact::efc_address);     // address in efc; -1: not included, -2-i: distance constraint i

  value_object<mjLROpt>("mjLROpt")
      .field("mode"       , &mjLROpt::mode)
      .field("useexisting", &mjLROpt::useexisting)
      .field("uselimit"   , &mjLROpt::uselimit)
      .field("accel"      , &mjLROpt::accel)      // target acceleration used to compute force
      .field("maxforce"   , &mjLROpt::maxforce)   // maximum force; 0: no limit
      .field("timeconst"  , &mjLROpt::timeconst)  // time constant for velocity reduction; min 0.01
      .field("timestep"   , &mjLROpt::timestep)   // simulation timestep; 0: use mjOption.timestep
      .field("inttotal"   , &mjLROpt::inttotal)   // total simulation time interval
      .field("interval"   , &mjLROpt::interval)   // evaluation time interval (at the end)
      .field("tolrange"   , &mjLROpt::tolrange);  // convergence tolerance (relative to range)

  value_object<mjOption>("mjOption")
      .field("timestep"            , &mjOption::timestep)          // timestep
      .field("apirate"             , &mjOption::apirate)           // update rate for remote API (Hz)
      .field("impratio"            , &mjOption::impratio)          // ratio of friction-to-normal contact impedance
      .field("tolerance"           , &mjOption::tolerance)         // main solver tolerance
      .field("noslip_tolerance"    , &mjOption::noslip_tolerance)  // noslip solver tolerance
      // .field("mpr_tolerance"       , &mjOption::mpr_tolerance)     // MPR solver tolerance
      //.field("gravity"           , &mjOption::gravity)           // gravitational acceleration
      //.field("wind"              , &mjOption::wind)              // wind (for lift, drag and viscosity)
      //.field("magnetic"          , &mjOption::magnetic)          // global magnetic flux
      .field("density"             , &mjOption::density)           // density of medium
      .field("viscosity"           , &mjOption::viscosity)         // viscosity of medium
      .field("o_margin"            , &mjOption::o_margin)          // margin
      //.field("o_solref"          , &mjOption::o_solref)          // solref
      //.field("o_solimp"          , &mjOption::o_solimp)          // solimp
      .field("integrator"          , &mjOption::integrator)        // integration mode (mjtIntegrator)
      // .field("collision"           , &mjOption::collision)         // collision mode (mjtCollision)
      .field("cone"                , &mjOption::cone)              // type of friction cone (mjtCone)
      .field("jacobian"            , &mjOption::jacobian)          // type of Jacobian (mjtJacobian)
      .field("solver"              , &mjOption::solver)            // solver algorithm (mjtSolver)
      .field("iterations"          , &mjOption::iterations)        // maximum number of main solver iterations
      .field("noslip_iterations"   , &mjOption::noslip_iterations) // maximum number of noslip solver iterations
      // .field("mpr_iterations"      , &mjOption::mpr_iterations)    // maximum number of MPR solver iterations
      .field("disableflags"        , &mjOption::disableflags)      // bit flags for disabling standard features
      .field("enableflags"         , &mjOption::enableflags);      // bit flags for enabling optional features

  register_vector<mjContact>("vector<mjContact>");
}
