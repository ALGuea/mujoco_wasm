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
  int  nv() const { return m->nv             ; }
  int  nu() const { return m->nu; }
  int  na() const { return m->na; }
  int  nbody() const { return m->nbody; }
  int  nbvh() const { return m->nbvh; }
  int  nbvhstatic() const { return m->nbvhstatic; }
  int  njnt() const { return m->njnt; }
  int  ngeom () const { return m->ngeom ; }
  int  nsite () const { return m->nsite ; }
  int  ncam() const { return m->ncam; }
  int  nlight() const { return m->nlight; }
  int  nflex () const { return m->nflex ; }
  int  nflexvert() const { return m->nflexvert; }
  int  nflextexcoord() const { return m->nflextexcoord; }
  int  nmesh () const { return m->nmesh ; }
  int  nskin () const { return m->nskin ; }
  int  nskinvert() const { return m->nskinvert; }
  int  nskinface() const { return m->nskinface; }
  int  nskinbone() const { return m->nskinbone; }
  int  nskinbonevert() const { return m->nskinbonevert; }
  int  nmat() const { return m->nmat; }
  int  neq() const { return m->neq; }
  int  ntendon() const { return m->ntendon; }
  int  nwrap () const { return m->nwrap ; }
  int  nsensor() const { return m->nsensor; }
  int  nnames() const { return m->nnames; }
  int  npaths() const { return m->npaths; }
  int  ntree () const { return m->ntree ; }
  int  nsensordata() const { return m->nsensordata; }
  val  body_parentid          () const { return val(typed_memory_view(m->nbody, m->body_parentid )); }
  val  body_rootid            () const { return val(typed_memory_view(m->nbody, m->body_rootid )); }
  val  body_weldid            () const { return val(typed_memory_view(m->nbody, m->body_weldid )); }
  val  body_mocapid           () const { return val(typed_memory_view(m->nbody, m->body_mocapid )); }
  val  body_jntnum            () const { return val(typed_memory_view(m->nbody, m->body_jntnum )); }
  val  body_jntadr            () const { return val(typed_memory_view(m->nbody, m->body_jntadr )); }
  val  body_dofnum            () const { return val(typed_memory_view(m->nbody, m->body_dofnum )); }
  val  body_dofadr            () const { return val(typed_memory_view(m->nbody, m->body_dofadr )); }
  val  body_geomnum           () const { return val(typed_memory_view(m->nbody, m->body_geomnum )); }
  val  body_geomadr           () const { return val(typed_memory_view(m->nbody, m->body_geomadr )); }
  val  body_iquat             () const { return val(typed_memory_view(m->nbody, m->body_iquat )); }
  val  body_mass              () const { return val(typed_memory_view(m->nbody, m->body_mass )); }
  val  body_inertia           () const { return val(typed_memory_view(m->nbody, m->body_inertia )); }
  val  body_bvhadr            () const { return val(typed_memory_view(m->nbody, m->body_bvhadr )); }
  val  body_bvhnum            () const { return val(typed_memory_view(m->nbody, m->body_bvhnum )); }
  val  bvh_depth              () const { return val(typed_memory_view(m->nbvh, m->bvh_depth )); }
  val  bvh_child              () const { return val(typed_memory_view(m->nbvh, m->bvh_child )); }
  val  bvh_nodeid             () const { return val(typed_memory_view(m->nbvh, m->bvh_nodeid )); }
  val  bvh_aabb               () const { return val(typed_memory_view(m->nbvhstatic, m->bvh_aabb )); }
  val  jnt_type               () const { return val(typed_memory_view(m->njnt, m->jnt_type )); }
  val  jnt_bodyid             () const { return val(typed_memory_view(m->njnt, m->jnt_bodyid )); }
  val  jnt_group              () const { return val(typed_memory_view(m->njnt, m->jnt_group )); }
  val  geom_type              () const { return val(typed_memory_view(m->ngeom, m->geom_type )); }
  val  geom_contype           () const { return val(typed_memory_view(m->ngeom, m->geom_contype )); }
  val  geom_conaffinity       () const { return val(typed_memory_view(m->ngeom, m->geom_conaffinity )); }
  val  geom_bodyid            () const { return val(typed_memory_view(m->ngeom, m->geom_bodyid )); }
  val  geom_dataid            () const { return val(typed_memory_view(m->ngeom, m->geom_dataid )); }
  val  geom_matid             () const { return val(typed_memory_view(m->ngeom, m->geom_matid )); }
  val  geom_group             () const { return val(typed_memory_view(m->ngeom, m->geom_group )); }
  val  geom_size              () const { return val(typed_memory_view(m->ngeom, m->geom_size )); }
  val  geom_aabb              () const { return val(typed_memory_view(m->ngeom, m->geom_aabb )); }
  val  geom_rbound            () const { return val(typed_memory_view(m->ngeom, m->geom_rbound )); }
  val  geom_rgba              () const { return val(typed_memory_view(m->ngeom, m->geom_rgba )); }
  val  site_type              () const { return val(typed_memory_view(m->nsite, m->site_type )); }
  val  site_bodyid            () const { return val(typed_memory_view(m->nsite, m->site_bodyid )); }
  val  site_matid             () const { return val(typed_memory_view(m->nsite, m->site_matid )); }
  val  site_group             () const { return val(typed_memory_view(m->nsite, m->site_group )); }
  val  site_size              () const { return val(typed_memory_view(m->nsite, m->site_size )); }
  val  site_rgba              () const { return val(typed_memory_view(m->nsite, m->site_rgba )); }
  val  cam_sensorsize         () const { return val(typed_memory_view(m->ncam, m->cam_sensorsize )); }
  val  cam_intrinsic          () const { return val(typed_memory_view(m->ncam, m->cam_intrinsic )); }
  val  cam_fovy               () const { return val(typed_memory_view(m->ncam, m->cam_fovy )); }
  val  cam_ipd                () const { return val(typed_memory_view(m->ncam, m->cam_ipd )); }
  val  light_directional      () const { return val(typed_memory_view(m->nlight, m->light_directional )); }
  val  light_castshadow       () const { return val(typed_memory_view(m->nlight, m->light_castshadow )); }
  val  light_active           () const { return val(typed_memory_view(m->nlight, m->light_active )); }
  val  light_attenuation      () const { return val(typed_memory_view(m->nlight, m->light_attenuation )); }
  val  light_cutoff           () const { return val(typed_memory_view(m->nlight, m->light_cutoff )); }
  val  light_exponent         () const { return val(typed_memory_view(m->nlight, m->light_exponent )); }
  val  light_ambient          () const { return val(typed_memory_view(m->nlight, m->light_ambient )); }
  val  light_diffuse          () const { return val(typed_memory_view(m->nlight, m->light_diffuse )); }
  val  light_specular         () const { return val(typed_memory_view(m->nlight, m->light_specular )); }
  val  flex_dim               () const { return val(typed_memory_view(m->nflex, m->flex_dim )); }
  val  flex_matid             () const { return val(typed_memory_view(m->nflex, m->flex_matid )); }
  val  flex_group             () const { return val(typed_memory_view(m->nflex, m->flex_group )); }
  val  flex_vertadr           () const { return val(typed_memory_view(m->nflex, m->flex_vertadr )); }
  val  flex_vertnum           () const { return val(typed_memory_view(m->nflex, m->flex_vertnum )); }
  val  flex_elemadr           () const { return val(typed_memory_view(m->nflex, m->flex_elemadr )); }
  val  flex_elemnum           () const { return val(typed_memory_view(m->nflex, m->flex_elemnum )); }
  val  flex_elemdataadr       () const { return val(typed_memory_view(m->nflex, m->flex_elemdataadr )); }
  val  flex_shellnum          () const { return val(typed_memory_view(m->nflex, m->flex_shellnum )); }
  val  flex_shelldataadr      () const { return val(typed_memory_view(m->nflex, m->flex_shelldataadr )); }
  val  flex_texcoordadr       () const { return val(typed_memory_view(m->nflex, m->flex_texcoordadr )); }
  val  flex_elem              () const { return val(typed_memory_view(m->nflexelemdata, m->flex_elem )); }
  val  flex_elemlayer         () const { return val(typed_memory_view(m->nflexelem, m->flex_elemlayer )); }
  val  flex_shell             () const { return val(typed_memory_view(m->nflexshelldata, m->flex_shell )); }
  val  flex_radius            () const { return val(typed_memory_view(m->nflex, m->flex_radius )); }
  val  flex_flatskin          () const { return val(typed_memory_view(m->nflex, m->flex_flatskin )); }
  val  flex_bvhadr            () const { return val(typed_memory_view(m->nflex, m->flex_bvhadr )); }
  val  flex_bvhnum            () const { return val(typed_memory_view(m->nflex, m->flex_bvhnum )); }
  val  flex_rgba              () const { return val(typed_memory_view(m->nflex, m->flex_rgba )); }
  val  mesh_texcoordadr       () const { return val(typed_memory_view(m->nmesh, m->mesh_texcoordadr )); }
  val  mesh_bvhadr            () const { return val(typed_memory_view(m->nmesh, m->mesh_bvhadr )); }
  val  mesh_bvhnum            () const { return val(typed_memory_view(m->nmesh, m->mesh_bvhnum )); }
  val  mesh_graphadr          () const { return val(typed_memory_view(m->nmesh, m->mesh_graphadr )); }
  val  mesh_pathadr           () const { return val(typed_memory_view(m->nmesh, m->mesh_pathadr )); }
  val  skin_matid             () const { return val(typed_memory_view(m->nskin, m->skin_matid )); }
  val  skin_group             () const { return val(typed_memory_view(m->nskin, m->skin_group )); }
  val  skin_rgba              () const { return val(typed_memory_view(m->nskin, m->skin_rgba )); }
  val  skin_inflate           () const { return val(typed_memory_view(m->nskin, m->skin_inflate )); }
  val  skin_vertadr           () const { return val(typed_memory_view(m->nskin, m->skin_vertadr )); }
  val  skin_vertnum           () const { return val(typed_memory_view(m->nskin, m->skin_vertnum )); }
  val  skin_texcoordadr       () const { return val(typed_memory_view(m->nskin, m->skin_texcoordadr )); }
  val  skin_faceadr           () const { return val(typed_memory_view(m->nskin, m->skin_faceadr )); }
  val  skin_facenum           () const { return val(typed_memory_view(m->nskin, m->skin_facenum )); }
  val  skin_boneadr           () const { return val(typed_memory_view(m->nskin, m->skin_boneadr )); }
  val  skin_bonenum           () const { return val(typed_memory_view(m->nskin, m->skin_bonenum )); }
  val  skin_vert              () const { return val(typed_memory_view(m->nskinvert, m->skin_vert )); }
  val  skin_face              () const { return val(typed_memory_view(m->nskinface, m->skin_face )); }
  val  skin_bonevertadr       () const { return val(typed_memory_view(m->nskinbone, m->skin_bonevertadr )); }
  val  skin_bonevertnum       () const { return val(typed_memory_view(m->nskinbone, m->skin_bonevertnum )); }
  val  skin_bonebindpos       () const { return val(typed_memory_view(m->nskinbone, m->skin_bonebindpos )); }
  val  skin_bonebindquat      () const { return val(typed_memory_view(m->nskinbone, m->skin_bonebindquat )); }
  val  skin_bonebodyid        () const { return val(typed_memory_view(m->nskinbone, m->skin_bonebodyid )); }
  val  skin_bonevertid        () const { return val(typed_memory_view(m->nskinbonevert, m->skin_bonevertid )); }
  val  skin_bonevertweight    () const { return val(typed_memory_view(m->nskinbonevert, m->skin_bonevertweight )); }
  val  skin_pathadr           () const { return val(typed_memory_view(m->nskin, m->skin_pathadr )); }
  val  hfield_pathadr         () const { return val(typed_memory_view(m->nhfield, m->hfield_pathadr )); }
  val  tex_pathadr            () const { return val(typed_memory_view(m->ntex, m->tex_pathadr )); }
  val  mat_texid              () const { return val(typed_memory_view(m->nmat, m->mat_texid )); }
  val  mat_texuniform         () const { return val(typed_memory_view(m->nmat, m->mat_texuniform )); }
  val  mat_texrepeat          () const { return val(typed_memory_view(m->nmat, m->mat_texrepeat )); }
  val  mat_emission           () const { return val(typed_memory_view(m->nmat, m->mat_emission )); }
  val  mat_specular           () const { return val(typed_memory_view(m->nmat, m->mat_specular )); }
  val  mat_shininess          () const { return val(typed_memory_view(m->nmat, m->mat_shininess )); }
  val  mat_reflectance        () const { return val(typed_memory_view(m->nmat, m->mat_reflectance )); }
  val  mat_rgba               () const { return val(typed_memory_view(m->nmat, m->mat_rgba )); }
  val  eq_type                () const { return val(typed_memory_view(m->neq, m->eq_type )); }
  val  eq_obj1id              () const { return val(typed_memory_view(m->neq, m->eq_obj1id )); }
  val  eq_obj2id              () const { return val(typed_memory_view(m->neq, m->eq_obj2id )); }
  val  eq_data                () const { return val(typed_memory_view(m->neq, m->eq_data )); }
  val  tendon_num             () const { return val(typed_memory_view(m->ntendon, m->tendon_num )); }
  val  tendon_matid           () const { return val(typed_memory_view(m->ntendon, m->tendon_matid )); }
  val  tendon_group           () const { return val(typed_memory_view(m->ntendon, m->tendon_group )); }
  val  tendon_limited         () const { return val(typed_memory_view(m->ntendon, m->tendon_limited )); }
  val  tendon_width           () const { return val(typed_memory_view(m->ntendon, m->tendon_width )); }
  val  tendon_range           () const { return val(typed_memory_view(m->ntendon, m->tendon_range )); }
  val  tendon_stiffness       () const { return val(typed_memory_view(m->ntendon, m->tendon_stiffness )); }
  val  tendon_damping         () const { return val(typed_memory_view(m->ntendon, m->tendon_damping )); }
  val  tendon_frictionloss    () const { return val(typed_memory_view(m->ntendon, m->tendon_frictionloss )); }
  val  tendon_lengthspring    () const { return val(typed_memory_view(m->ntendon, m->tendon_lengthspring )); }
  val  tendon_rgba            () const { return val(typed_memory_view(m->ntendon, m->tendon_rgba )); }
  val  actuator_trntype       () const { return val(typed_memory_view(m->nu, m->actuator_trntype )); }
  val  actuator_dyntype       () const { return val(typed_memory_view(m->nu, m->actuator_dyntype )); }
  val  actuator_trnid         () const { return val(typed_memory_view(m->nu, m->actuator_trnid )); }
  val  actuator_actadr        () const { return val(typed_memory_view(m->nu, m->actuator_actadr )); }
  val  actuator_actnum        () const { return val(typed_memory_view(m->nu, m->actuator_actnum )); }
  val  actuator_group         () const { return val(typed_memory_view(m->nu, m->actuator_group )); }
  val  actuator_ctrllimited   () const { return val(typed_memory_view(m->nu, m->actuator_ctrllimited )); }
  val  actuator_actlimited    () const { return val(typed_memory_view(m->nu, m->actuator_actlimited )); }
  val  actuator_ctrlrange     () const { return val(typed_memory_view(m->nu, m->actuator_ctrlrange )); }
  val  actuator_actrange      () const { return val(typed_memory_view(m->nu, m->actuator_actrange )); }
  val  actuator_cranklength   () const { return val(typed_memory_view(m->nu, m->actuator_cranklength )); }
  val  sensor_type            () const { return val(typed_memory_view(m->nsensor, m->sensor_type )); }
  val  sensor_objid           () const { return val(typed_memory_view(m->nsensor, m->sensor_objid )); }
  val  sensor_adr             () const { return val(typed_memory_view(m->nsensor, m->sensor_adr )); }
  val  name_bodyadr           () const { return val(typed_memory_view(m->nbody, m->name_bodyadr )); }
  val  name_jntadr            () const { return val(typed_memory_view(m->njnt, m->name_jntadr )); }
  val  name_geomadr           () const { return val(typed_memory_view(m->ngeom, m->name_geomadr )); }
  val  name_siteadr           () const { return val(typed_memory_view(m->nsite, m->name_siteadr )); }
  val  name_camadr            () const { return val(typed_memory_view(m->ncam, m->name_camadr )); }
  val  name_lightadr          () const { return val(typed_memory_view(m->nlight, m->name_lightadr )); }
  val  name_eqadr             () const { return val(typed_memory_view(m->neq, m->name_eqadr )); }
  val  name_tendonadr         () const { return val(typed_memory_view(m->ntendon, m->name_tendonadr )); }
  val  name_actuatoradr       () const { return val(typed_memory_view(m->nu, m->name_actuatoradr )); }
  val  names                  () const { return val(typed_memory_view(m->nnames, m->names )); }
  val  paths                  () const { return val(typed_memory_view(m->npaths, m->paths )); }

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
  val  qpos                   () const { return val(typed_memory_view(_model->ptr()->nq * 1, _state->ptr()->qpos)); }
  val  qvel                   () const { return val(typed_memory_view(_model->ptr()->nv* 1, _state->ptr()->qvel)); }
  val  act                    () const { return val(typed_memory_view(_model->ptr()->na, _state->ptr()->act )); }
  val  ctrl                   () const { return val(typed_memory_view(_model->ptr()->nu, _state->ptr()->ctrl )); }
  val  xfrc_applied           () const { return val(typed_memory_view(_model->ptr()->nbody, _state->ptr()->xfrc_applied )); }
  val  eq_active              () const { return val(typed_memory_view(_model->ptr()->neq, _state->ptr()->eq_active )); }
  val  sensordata             () const { return val(typed_memory_view(_model->ptr()->nsensordata, _state->ptr()->sensordata )); }
  val  xpos                   () const { return val(typed_memory_view(_model->ptr()->nbody, _state->ptr()->xpos )); }
  val  xquat                  () const { return val(typed_memory_view(_model->ptr()->nbody, _state->ptr()->xquat )); }
  val  xmat                   () const { return val(typed_memory_view(_model->ptr()->nbody, _state->ptr()->xmat )); }
  val  xipos                  () const { return val(typed_memory_view(_model->ptr()->nbody, _state->ptr()->xipos )); }
  val  ximat                  () const { return val(typed_memory_view(_model->ptr()->nbody, _state->ptr()->ximat )); }
  val  xanchor                () const { return val(typed_memory_view(_model->ptr()->njnt, _state->ptr()->xanchor )); }
  val  xaxis                  () const { return val(typed_memory_view(_model->ptr()->njnt, _state->ptr()->xaxis )); }
  val  geom_xpos              () const { return val(typed_memory_view(_model->ptr()->ngeom, _state->ptr()->geom_xpos )); }
  val  geom_xmat              () const { return val(typed_memory_view(_model->ptr()->ngeom, _state->ptr()->geom_xmat )); }
  val  site_xpos              () const { return val(typed_memory_view(_model->ptr()->nsite, _state->ptr()->site_xpos )); }
  val  site_xmat              () const { return val(typed_memory_view(_model->ptr()->nsite, _state->ptr()->site_xmat )); }
  val  cam_xpos               () const { return val(typed_memory_view(_model->ptr()->ncam, _state->ptr()->cam_xpos )); }
  val  cam_xmat               () const { return val(typed_memory_view(_model->ptr()->ncam, _state->ptr()->cam_xmat )); }
  val  light_xpos             () const { return val(typed_memory_view(_model->ptr()->nlight, _state->ptr()->light_xpos )); }
  val  light_xdir             () const { return val(typed_memory_view(_model->ptr()->nlight, _state->ptr()->light_xdir )); }
  val  subtree_com            () const { return val(typed_memory_view(_model->ptr()->nbody, _state->ptr()->subtree_com )); }
  val  flexvert_xpos          () const { return val(typed_memory_view(_model->ptr()->nflexvert, _state->ptr()->flexvert_xpos )); }
  val  ten_wrapadr            () const { return val(typed_memory_view(_model->ptr()->ntendon, _state->ptr()->ten_wrapadr )); }
  val  ten_wrapnum            () const { return val(typed_memory_view(_model->ptr()->ntendon, _state->ptr()->ten_wrapnum )); }
  val  ten_length             () const { return val(typed_memory_view(_model->ptr()->ntendon, _state->ptr()->ten_length )); }
  val  wrap_obj               () const { return val(typed_memory_view(_model->ptr()->nwrap, _state->ptr()->wrap_obj )); }
  val  wrap_xpos              () const { return val(typed_memory_view(_model->ptr()->nwrap, _state->ptr()->wrap_xpos )); }
  val  bvh_aabb_dyn           () const { return val(typed_memory_view(_model->ptr()->nbvhdynamic, _state->ptr()->bvh_aabb_dyn )); }
  val  bvh_active             () const { return val(typed_memory_view(_model->ptr()->nbvh, _state->ptr()->bvh_active )); }
  val  contact                () const { return val(typed_memory_view(_model->ptr()->ncon, _state->ptr()->contact )); }
  val  efc_type               () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_type )); }
  val  efc_id                 () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_id )); }
  val  efc_J_rownnz           () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_J_rownnz )); }
  val  efc_J_rowadr           () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_J_rowadr )); }
  val  efc_J_rowsuper         () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_J_rowsuper )); }
  val  efc_J_colind           () const { return val(typed_memory_view(_model->ptr()->nnzJ, _state->ptr()->efc_J_colind )); }
  val  efc_JT_rownnz          () const { return val(typed_memory_view(_model->ptr()->nv, _state->ptr()->efc_JT_rownnz )); }
  val  efc_JT_rowadr          () const { return val(typed_memory_view(_model->ptr()->nv, _state->ptr()->efc_JT_rowadr )); }
  val  efc_JT_rowsuper        () const { return val(typed_memory_view(_model->ptr()->nv, _state->ptr()->efc_JT_rowsuper )); }
  val  efc_JT_colind          () const { return val(typed_memory_view(_model->ptr()->nnzJ, _state->ptr()->efc_JT_colind )); }
  val  efc_J                  () const { return val(typed_memory_view(_model->ptr()->nnzJ, _state->ptr()->efc_J )); }
  val  efc_JT                 () const { return val(typed_memory_view(_model->ptr()->nnzJ, _state->ptr()->efc_JT )); }
  val  efc_pos                () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_pos )); }
  val  efc_margin             () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_margin )); }
  val  efc_frictionloss       () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_frictionloss )); }
  val  efc_diagApprox         () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_diagApprox )); }
  val  efc_KBIP               () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_KBIP )); }
  val  efc_D                  () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_D )); }
  val  efc_R                  () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_R )); }
  val  tendon_efcadr          () const { return val(typed_memory_view(_model->ptr()->ntendon, _state->ptr()->tendon_efcadr )); }
  val  efc_vel                () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_vel )); }
  val  efc_aref               () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_aref )); }
  val  efc_b                  () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_b )); }
  val  efc_force              () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_force )); }
  val  efc_state              () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_state )); }
  val  efc_AR_rownnz          () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_AR_rownnz )); }
  val  efc_AR_rowadr          () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_AR_rowadr )); }
  val  efc_AR_colind          () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_AR_colind )); }
  val  efc_AR                 () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_AR )); }
  val  dof_island             () const { return val(typed_memory_view(_model->ptr()->nv, _state->ptr()->dof_island )); }
  val  island_dofnum          () const { return val(typed_memory_view(_model->ptr()->nisland, _state->ptr()->island_dofnum )); }
  val  island_dofadr          () const { return val(typed_memory_view(_model->ptr()->nisland, _state->ptr()->island_dofadr )); }
  val  island_dofind          () const { return val(typed_memory_view(_model->ptr()->nv, _state->ptr()->island_dofind )); }
  val  dof_islandind          () const { return val(typed_memory_view(_model->ptr()->nv, _state->ptr()->dof_islandind )); }
  val  efc_island             () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->efc_island )); }
  val  island_efcnum          () const { return val(typed_memory_view(_model->ptr()->nisland, _state->ptr()->island_efcnum )); }
  val  island_efcadr          () const { return val(typed_memory_view(_model->ptr()->nisland, _state->ptr()->island_efcadr )); }
  val  island_efcind          () const { return val(typed_memory_view(_model->ptr()->nefc, _state->ptr()->island_efcind )); }
  val  narena() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->narena)); }
  val  nbuffer() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->nbuffer)); }
  val  nplugin() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->nplugin)); }
  val  pstack() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->pstack)); }
  val  pbase () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->pbase )); }
  val  parena() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->parena)); }
  val  maxuse_stack() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->maxuse_stack)); }
  val  maxuse_arena() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->maxuse_arena)); }
  val  maxuse_con  () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->maxuse_con  )); }
  val  maxuse_efc  () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->maxuse_efc  )); }
  val  solver_nisland() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->solver_nisland)); }
  val  ne    () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->ne    )); }
  val  nf    () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->nf    )); }
  val  nl    () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->nl    )); }
  val  nefc  () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->nefc  )); }
  val  nnzJ  () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->nnzJ  )); }
  val  ncon  () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->ncon  )); }
  val  nisland() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->nisland)); }
  val  time  () const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->time  )); }
  val  threadpool() const { return val(typed_memory_view(_model->ptr()->1, _state->ptr()->threadpool         ) ); }
  val  maxuse_threadstack     () const { return val(typed_memory_view(_model->ptr()->mjMAXTHREAD, _state->ptr()->maxuse_threadstack )); }
  val  warning                () const { return val(typed_memory_view(_model->ptr()->mjNWARNING, _state->ptr()->warning )); }
  val  timer                  () const { return val(typed_memory_view(_model->ptr()->mjNTIMER, _state->ptr()->timer )); }
  val  solver                 () const { return val(typed_memory_view(_model->ptr()->mjNILSAND, _state->ptr()->solver )); }
  val  solver_niter           () const { return val(typed_memory_view(_model->ptr()->mjNISLAND, _state->ptr()->solver_niter )); }
  val  solver_nnz             () const { return val(typed_memory_view(_model->ptr()->mjNISLAND, _state->ptr()->solver_nnz )); }
  val  solver_fwdinv          () const { return val(typed_memory_view(_model->ptr()->2, _state->ptr()->solver_fwdinv )); }
  val  energy                 () const { return val(typed_memory_view(_model->ptr()->2, _state->ptr()->energy )); }
  void   freeLastXML         (                    ) { return mj_freeLastXML              (                    ); }
  void   step                (                    ) { return mj_step                     (_model->ptr(), _state->ptr()); }
  void   step1               (                    ) { return mj_step1                    (_model->ptr(), _state->ptr()); }
  void   step2               (                    ) { return mj_step2                    (_model->ptr(), _state->ptr()); }
  void   forward             (                    ) { return mj_forward                  (_model->ptr(), _state->ptr()); }
  void   inverse             (                    ) { return mj_inverse                  (_model->ptr(), _state->ptr()); }
  void   forwardSkip         (int skipstage, int skipsensor) { return mj_forwardSkip              (_model->ptr(), _state->ptr(), skipstage, skipsensor); }
  void   inverseSkip         (int skipstage, int skipsensor) { return mj_inverseSkip              (_model->ptr(), _state->ptr(), skipstage, skipsensor); }
  void   defaultSolRefImp    (val solref, val solimp) { return mj_defaultSolRefImp         (reinterpret_cast<mjtNum*>(solref["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(solimp["byteOffset"].as<int>())); }
  int    sizeModel           (                    ) { return mj_sizeModel                (_model->ptr()       ); }
  void   resetData           (                    ) { return mj_resetData                (_model->ptr(), _state->ptr()); }
  void   resetDataDebug      (unsigned char debug_value) { return mj_resetDataDebug           (_model->ptr(), _state->ptr(), debug_value); }
  void   resetDataKeyframe   (int key             ) { return mj_resetDataKeyframe        (_model->ptr(), _state->ptr(), key); }
  void   markStack           (                    ) { return mj_markStack                (_state->ptr()       ); }
  void   freeStack           (                    ) { return mj_freeStack                (_state->ptr()       ); }
  void   deleteData          (                    ) { return mj_deleteData               (_state->ptr()       ); }
  void   resetCallbacks      (                    ) { return mj_resetCallbacks           (                    ); }
  void   printFormattedModel (std::string filename, std::string float_format) { return mj_printFormattedModel      (_model->ptr(), filename.c_str(), float_format.c_str()); }
  void   printModel          (std::string filename) { return mj_printModel               (_model->ptr(), filename.c_str()); }
  void   printFormattedData  (std::string filename, std::string float_format) { return mj_printFormattedData       (_model->ptr(), _state->ptr(), filename.c_str(), float_format.c_str()); }
  void   printData           (std::string filename) { return mj_printData                (_model->ptr(), _state->ptr(), filename.c_str()); }
  void   _printMat           (val mat, int nr, int nc) { return mju_printMat                (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), nr, nc); }
  void   fwdPosition         (                    ) { return mj_fwdPosition              (_model->ptr(), _state->ptr()); }
  void   fwdVelocity         (                    ) { return mj_fwdVelocity              (_model->ptr(), _state->ptr()); }
  void   fwdActuation        (                    ) { return mj_fwdActuation             (_model->ptr(), _state->ptr()); }
  void   fwdAcceleration     (                    ) { return mj_fwdAcceleration          (_model->ptr(), _state->ptr()); }
  void   fwdConstraint       (                    ) { return mj_fwdConstraint            (_model->ptr(), _state->ptr()); }
  void   Euler               (                    ) { return mj_Euler                    (_model->ptr(), _state->ptr()); }
  void   RungeKutta          (int N               ) { return mj_RungeKutta               (_model->ptr(), _state->ptr(), N); }
  void   implicit            (                    ) { return mj_implicit                 (_model->ptr(), _state->ptr()); }
  void   invPosition         (                    ) { return mj_invPosition              (_model->ptr(), _state->ptr()); }
  void   invVelocity         (                    ) { return mj_invVelocity              (_model->ptr(), _state->ptr()); }
  void   invConstraint       (                    ) { return mj_invConstraint            (_model->ptr(), _state->ptr()); }
  void   compareFwdInv       (                    ) { return mj_compareFwdInv            (_model->ptr(), _state->ptr()); }
  void   sensorPos           (                    ) { return mj_sensorPos                (_model->ptr(), _state->ptr()); }
  void   sensorVel           (                    ) { return mj_sensorVel                (_model->ptr(), _state->ptr()); }
  void   sensorAcc           (                    ) { return mj_sensorAcc                (_model->ptr(), _state->ptr()); }
  void   energyPos           (                    ) { return mj_energyPos                (_model->ptr(), _state->ptr()); }
  void   energyVel           (                    ) { return mj_energyVel                (_model->ptr(), _state->ptr()); }
  void   checkPos            (                    ) { return mj_checkPos                 (_model->ptr(), _state->ptr()); }
  void   checkVel            (                    ) { return mj_checkVel                 (_model->ptr(), _state->ptr()); }
  void   checkAcc            (                    ) { return mj_checkAcc                 (_model->ptr(), _state->ptr()); }
  void   kinematics          (                    ) { return mj_kinematics               (_model->ptr(), _state->ptr()); }
  void   comPos              (                    ) { return mj_comPos                   (_model->ptr(), _state->ptr()); }
  void   camlight            (                    ) { return mj_camlight                 (_model->ptr(), _state->ptr()); }
  void   flex                (                    ) { return mj_flex                     (_model->ptr(), _state->ptr()); }
  void   tendon              (                    ) { return mj_tendon                   (_model->ptr(), _state->ptr()); }
  void   transmission        (                    ) { return mj_transmission             (_model->ptr(), _state->ptr()); }
  void   crbCalculate        (                    ) { return mj_crb                      (_model->ptr(), _state->ptr()); }
  void   factorM             (                    ) { return mj_factorM                  (_model->ptr(), _state->ptr()); }
  void   solveM              (val x, val y, int n ) { return mj_solveM                   (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(y["byteOffset"].as<int>()), n); }
  void   solveM2             (val x, val y, int n ) { return mj_solveM2                  (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(y["byteOffset"].as<int>()), n); }
  void   comVel              (                    ) { return mj_comVel                   (_model->ptr(), _state->ptr()); }
  void   passive             (                    ) { return mj_passive                  (_model->ptr(), _state->ptr()); }
  void   subtreeVel          (                    ) { return mj_subtreeVel               (_model->ptr(), _state->ptr()); }
  void   rne                 (int flg_acc, val result) { return mj_rne                      (_model->ptr(), _state->ptr(), flg_acc, reinterpret_cast<mjtNum*>(result["byteOffset"].as<int>())); }
  void   rnePostConstraint   (                    ) { return mj_rnePostConstraint        (_model->ptr(), _state->ptr()); }
  void   collision           (                    ) { return mj_collision                (_model->ptr(), _state->ptr()); }
  void   makeConstraint      (                    ) { return mj_makeConstraint           (_model->ptr(), _state->ptr()); }
  void   island              (                    ) { return mj_island                   (_model->ptr(), _state->ptr()); }
  void   projectConstraint   (                    ) { return mj_projectConstraint        (_model->ptr(), _state->ptr()); }
  void   referenceConstraint (                    ) { return mj_referenceConstraint      (_model->ptr(), _state->ptr()); }
  int    stateSize           (unsigned int spec   ) { return mj_stateSize                (_model->ptr(), spec ); }
  void   setState            (val state, unsigned int spec) { return mj_setState                 (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(state["byteOffset"].as<int>()), spec); }
  int    isPyramidal         (                    ) { return mj_isPyramidal              (_model->ptr()       ); }
  int    isSparse            (                    ) { return mj_isSparse                 (_model->ptr()       ); }
  int    isDual              (                    ) { return mj_isDual                   (_model->ptr()       ); }
  void   jacSubtreeCom       (val jacp, int body  ) { return mj_jacSubtreeCom            (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(jacp["byteOffset"].as<int>()), body); }
  int    name2id             (int type, std::string name) { return mj_name2id                  (_model->ptr(), type, name.c_str()); }
  std::string id2name             (int type, int id    ) { return std::string(mj_id2name                  (_model->ptr(), type, id)); }
  void   fullM               (val dst, val M      ) { return mj_fullM                    (_model->ptr(), reinterpret_cast<mjtNum*>(dst["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(M["byteOffset"].as<int>())); }
  void   differentiatePos    (val qvel, mjtNum dt, val qpos1, val qpos2) { return mj_differentiatePos         (_model->ptr(), reinterpret_cast<mjtNum*>(qvel["byteOffset"].as<int>()), dt, reinterpret_cast<mjtNum*>(qpos1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(qpos2["byteOffset"].as<int>())); }
  void   integratePos        (val qpos, val qvel, mjtNum dt) { return mj_integratePos             (_model->ptr(), reinterpret_cast<mjtNum*>(qpos["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(qvel["byteOffset"].as<int>()), dt); }
  void   normalizeQuat       (val qpos            ) { return mj_normalizeQuat            (_model->ptr(), reinterpret_cast<mjtNum*>(qpos["byteOffset"].as<int>())); }
  mjtNum getTotalmass        (                    ) { return mj_getTotalmass             (_model->ptr()       ); }
  std::string getPluginConfig     (int plugin_id, std::string attrib) { return std::string(mj_getPluginConfig          (_model->ptr(), plugin_id, attrib.c_str())); }
  void   loadPluginLibrary   (std::string path    ) { return mj_loadPluginLibrary        (path.c_str()        ); }
  int    version             (                    ) { return mj_version                  (                    ); }
  std::string versionString       (                    ) { return std::string(mj_versionString            (                    )); }
  void   _rectangle          (mjrRect viewport, float r, float g, float b, float a) { return mjr_rectangle               (viewport, r, g, b, a); }
  void   _finish             (                    ) { return mjr_finish                  (                    ); }
  int    _getError           (                    ) { return mjr_getError                (                    ); }
  mjuiThemeSpacing i_themeSpacing      (int ind             ) { return mjui_themeSpacing           (ind                 ); }
  mjuiThemeColor i_themeColor        (int ind             ) { return mjui_themeColor             (ind                 ); }
  void   _error              (std::string msg     ) { return mju_error                   (msg.c_str()         ); }
  void   _error_i            (std::string msg, int i) { return mju_error_i                 (msg.c_str(), i      ); }
  void   _error_s            (std::string msg, std::string text) { return mju_error_s                 (msg.c_str(), text.c_str()); }
  void   _warning            (std::string msg     ) { return mju_warning                 (msg.c_str()         ); }
  void   _warning_i          (std::string msg, int i) { return mju_warning_i               (msg.c_str(), i      ); }
  void   _warning_s          (std::string msg, std::string text) { return mju_warning_s               (msg.c_str(), text.c_str()); }
  void   _clearHandlers      (                    ) { return mju_clearHandlers           (                    ); }
  void   warning             (int warning, int info) { return mj_warning                  (_state->ptr(), warning, info); }
  void   _writeLog           (std::string type, std::string msg) { return mju_writeLog                (type.c_str(), msg.c_str()); }
  void   _zero               (val res, int n      ) { return mju_zero                    (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  void   _fill               (val res, mjtNum val, int n) { return mju_fill                    (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), val, n); }
  void   _copy               (val res, val vec, int n) { return mju_copy                    (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  mjtNum _sum                (val vec, int n      ) { return mju_sum                     (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  mjtNum _L1                 (val vec, int n      ) { return mju_L1                      (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void   _scl                (val res, val vec, mjtNum scl, int n) { return mju_scl                     (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), scl, n); }
  void   _add                (val res, val vec1, val vec2, int n) { return mju_add                     (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void   _sub                (val res, val vec1, val vec2, int n) { return mju_sub                     (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void   _addTo              (val res, val vec, int n) { return mju_addTo                   (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void   _subFrom            (val res, val vec, int n) { return mju_subFrom                 (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void   _addToScl           (val res, val vec, mjtNum scl, int n) { return mju_addToScl                (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), scl, n); }
  void   _addScl             (val res, val vec1, val vec2, mjtNum scl, int n) { return mju_addScl                  (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), scl, n); }
  mjtNum _normalize          (val res, int n      ) { return mju_normalize               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  mjtNum _norm               (val res, int n      ) { return mju_norm                    (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  mjtNum _dot                (val vec1, val vec2, int n) { return mju_dot                     (reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void   _mulMatVec          (val res, val mat, val vec, int nr, int nc) { return mju_mulMatVec               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), nr, nc); }
  void   _mulMatTVec         (val res, val mat, val vec, int nr, int nc) { return mju_mulMatTVec              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), nr, nc); }
  mjtNum _mulVecMatVec       (val vec1, val mat, val vec2, int n) { return mju_mulVecMatVec            (reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void   _transpose          (val res, val mat, int nr, int nc) { return mju_transpose               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), nr, nc); }
  void   _symmetrize         (val res, val mat, int n) { return mju_symmetrize              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n); }
  void   _eye                (val mat, int n      ) { return mju_eye                     (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n); }
  void   _mulMatMat          (val res, val mat1, val mat2, int r1, int c1, int c2) { return mju_mulMatMat               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, c2); }
  void   _mulMatMatT         (val res, val mat1, val mat2, int r1, int c1, int r2) { return mju_mulMatMatT              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, r2); }
  void   _mulMatTMat         (val res, val mat1, val mat2, int r1, int c1, int c2) { return mju_mulMatTMat              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, c2); }
  void   _sqrMatTD           (val res, val mat, val diag, int nr, int nc) { return mju_sqrMatTD                (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(diag["byteOffset"].as<int>()), nr, nc); }
  int    _cholFactor         (val mat, int n, mjtNum mindiag) { return mju_cholFactor              (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n, mindiag); }
  void   _cholSolve          (val res, val mat, val vec, int n) { return mju_cholSolve               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  int    _cholUpdate         (val mat, val x, int n, int flg_plus) { return mju_cholUpdate              (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), n, flg_plus); }
  mjtNum _cholFactorBand     (val mat, int ntotal, int nband, int ndense, mjtNum diagadd, mjtNum diagmul) { return mju_cholFactorBand          (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), ntotal, nband, ndense, diagadd, diagmul); }
  void   _cholSolveBand      (val res, val mat, val vec, int ntotal, int nband, int ndense) { return mju_cholSolveBand           (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), ntotal, nband, ndense); }
  void   _band2Dense         (val res, val mat, int ntotal, int nband, int ndense, mjtByte flg_sym) { return mju_band2Dense              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), ntotal, nband, ndense, flg_sym); }
  void   _dense2Band         (val res, val mat, int ntotal, int nband, int ndense) { return mju_dense2Band              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), ntotal, nband, ndense); }
  void   _bandMulMatVec      (val res, val mat, val vec, int ntotal, int nband, int ndense, int nvec, mjtByte flg_sym) { return mju_bandMulMatVec           (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), ntotal, nband, ndense, nvec, flg_sym); }
  int    _bandDiag           (int i, int ntotal, int nband, int ndense) { return mju_bandDiag                (i, ntotal, nband, ndense); }
  void   _encodePyramid      (val pyramid, val force, val mu, int dim) { return mju_encodePyramid           (reinterpret_cast<mjtNum*>(pyramid["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(force["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mu["byteOffset"].as<int>()), dim); }
  void   _decodePyramid      (val force, val pyramid, val mu, int dim) { return mju_decodePyramid           (reinterpret_cast<mjtNum*>(force["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(pyramid["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mu["byteOffset"].as<int>()), dim); }
  mjtNum _springDamper       (mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt) { return mju_springDamper            (pos0, vel0, Kp, Kv, dt); }
  mjtNum _min                (mjtNum a, mjtNum b  ) { return mju_min                     (a, b                ); }
  mjtNum _max                (mjtNum a, mjtNum b  ) { return mju_max                     (a, b                ); }
  mjtNum _clip               (mjtNum x, mjtNum min, mjtNum max) { return mju_clip                    (x, min, max         ); }
  mjtNum _sign               (mjtNum x            ) { return mju_sign                    (x                   ); }
  int    _round              (mjtNum x            ) { return mju_round                   (x                   ); }
  std::string _type2Str           (int type            ) { return std::string(mju_type2Str                (type                )); }
  int    _str2Type           (std::string str     ) { return mju_str2Type                (str.c_str()         ); }
  std::string _writeNumBytes      (size_t nbytes       ) { return std::string(mju_writeNumBytes           (nbytes              )); }
  std::string _warningText        (int warning, size_t info) { return std::string(mju_warningText             (warning, info       )); }
  int    _isBad              (mjtNum x            ) { return mju_isBad                   (x                   ); }
  int    _isZero             (val vec, int n      ) { return mju_isZero                  (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  mjtNum _standardNormal     (val num2            ) { return mju_standardNormal          (reinterpret_cast<mjtNum*>(num2["byteOffset"].as<int>())); }
  void   _insertionSort      (val list, int n     ) { return mju_insertionSort           (reinterpret_cast<mjtNum*>(list["byteOffset"].as<int>()), n); }
  mjtNum _Halton             (int index, int base ) { return mju_Halton                  (index, base         ); }
  mjtNum _sigmoid            (mjtNum x            ) { return mju_sigmoid                 (x                   ); }
  void   _transitionFD       (mjtNum eps, mjtByte flg_centered, val A, val B, val C, val D) { return mjd_transitionFD            (_model->ptr(), _state->ptr(), eps, flg_centered, reinterpret_cast<mjtNum*>(A["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(B["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(C["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(D["byteOffset"].as<int>())); }
  void   _inverseFD          (mjtNum eps, mjtByte flg_actuation, val DfDq, val DfDv, val DfDa, val DsDq, val DsDv, val DsDa, val DmDq) { return mjd_inverseFD               (_model->ptr(), _state->ptr(), eps, flg_actuation, reinterpret_cast<mjtNum*>(DfDq["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(DfDv["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(DfDa["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(DsDq["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(DsDv["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(DsDa["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(DmDq["byteOffset"].as<int>())); }
  int    _pluginCount        (                    ) { return mjp_pluginCount             (                    ); }
  int    _resourceProviderCount(                    ) { return mjp_resourceProviderCount   (                    ); }


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
  enum_<mjtDisableBit>("mjtDisableBit")
      .value("mjDSBL_CONSTRAINT"      , mjtDisableBit            ::mjDSBL_CONSTRAINT        )
      .value("mjDSBL_EQUALITY"        , mjtDisableBit            ::mjDSBL_EQUALITY          )
      .value("mjDSBL_FRICTIONLOSS"    , mjtDisableBit            ::mjDSBL_FRICTIONLOSS      )
      .value("mjDSBL_LIMIT"           , mjtDisableBit            ::mjDSBL_LIMIT             )
      .value("mjDSBL_CONTACT"         , mjtDisableBit            ::mjDSBL_CONTACT           )
      .value("mjDSBL_PASSIVE"         , mjtDisableBit            ::mjDSBL_PASSIVE           )
      .value("mjDSBL_GRAVITY"         , mjtDisableBit            ::mjDSBL_GRAVITY           )
      .value("mjDSBL_CLAMPCTRL"       , mjtDisableBit            ::mjDSBL_CLAMPCTRL         )
      .value("mjDSBL_WARMSTART"       , mjtDisableBit            ::mjDSBL_WARMSTART         )
      .value("mjDSBL_FILTERPARENT"    , mjtDisableBit            ::mjDSBL_FILTERPARENT      )
      .value("mjDSBL_ACTUATION"       , mjtDisableBit            ::mjDSBL_ACTUATION         )
      .value("mjDSBL_REFSAFE"         , mjtDisableBit            ::mjDSBL_REFSAFE           )
      .value("mjDSBL_SENSOR"          , mjtDisableBit            ::mjDSBL_SENSOR            )
      .value("mjDSBL_MIDPHASE"        , mjtDisableBit            ::mjDSBL_MIDPHASE          )
      .value("mjDSBL_EULERDAMP"       , mjtDisableBit            ::mjDSBL_EULERDAMP         )
      .value("mjNDISABLE"             , mjtDisableBit            ::mjNDISABLE               )
  ;
  enum_<mjtEnableBit>("mjtEnableBit")
      .value("mjENBL_OVERRIDE"        , mjtEnableBit             ::mjENBL_OVERRIDE          )
      .value("mjENBL_ENERGY"          , mjtEnableBit             ::mjENBL_ENERGY            )
      .value("mjENBL_FWDINV"          , mjtEnableBit             ::mjENBL_FWDINV            )
      .value("mjENBL_INVDISCRETE"     , mjtEnableBit             ::mjENBL_INVDISCRETE       )
      .value("mjENBL_SENSORNOISE"     , mjtEnableBit             ::mjENBL_SENSORNOISE       )
      .value("mjENBL_MULTICCD"        , mjtEnableBit             ::mjENBL_MULTICCD          )
      .value("mjENBL_ISLAND"          , mjtEnableBit             ::mjENBL_ISLAND            )
      .value("mjNENABLE"              , mjtEnableBit             ::mjNENABLE                )
  ;
  enum_<mjtJoint>("mjtJoint")
      .value("mjJNT_FREE"             , mjtJoint                 ::mjJNT_FREE               )
      .value("mjJNT_BALL"             , mjtJoint                 ::mjJNT_BALL               )
      .value("mjJNT_SLIDE"            , mjtJoint                 ::mjJNT_SLIDE              )
      .value("mjJNT_HINGE"            , mjtJoint                 ::mjJNT_HINGE              )
  ;
  enum_<mjtGeom>("mjtGeom")
      .value("mjGEOM_PLANE"           , mjtGeom                  ::mjGEOM_PLANE             )
      .value("mjGEOM_HFIELD"          , mjtGeom                  ::mjGEOM_HFIELD            )
      .value("mjGEOM_SPHERE"          , mjtGeom                  ::mjGEOM_SPHERE            )
      .value("mjGEOM_CAPSULE"         , mjtGeom                  ::mjGEOM_CAPSULE           )
      .value("mjGEOM_ELLIPSOID"       , mjtGeom                  ::mjGEOM_ELLIPSOID         )
      .value("mjGEOM_CYLINDER"        , mjtGeom                  ::mjGEOM_CYLINDER          )
      .value("mjGEOM_BOX"             , mjtGeom                  ::mjGEOM_BOX               )
      .value("mjGEOM_MESH"            , mjtGeom                  ::mjGEOM_MESH              )
      .value("mjGEOM_SDF"             , mjtGeom                  ::mjGEOM_SDF               )
      .value("mjNGEOMTYPES"           , mjtGeom                  ::mjNGEOMTYPES             )
      .value("mjGEOM_ARROW"           , mjtGeom                  ::mjGEOM_ARROW             )
      .value("mjGEOM_ARROW1"          , mjtGeom                  ::mjGEOM_ARROW1            )
      .value("mjGEOM_ARROW2"          , mjtGeom                  ::mjGEOM_ARROW2            )
      .value("mjGEOM_LINE"            , mjtGeom                  ::mjGEOM_LINE              )
      .value("mjGEOM_LINEBOX"         , mjtGeom                  ::mjGEOM_LINEBOX           )
      .value("mjGEOM_FLEX"            , mjtGeom                  ::mjGEOM_FLEX              )
      .value("mjGEOM_SKIN"            , mjtGeom                  ::mjGEOM_SKIN              )
      .value("mjGEOM_LABEL"           , mjtGeom                  ::mjGEOM_LABEL             )
      .value("mjGEOM_TRIANGLE"        , mjtGeom                  ::mjGEOM_TRIANGLE          )
      .value("mjGEOM_NONE"            , mjtGeom                  ::mjGEOM_NONE              )
  ;
  enum_<mjtCamLight>("mjtCamLight")
      .value("mjCAMLIGHT_FIXED"       , mjtCamLight              ::mjCAMLIGHT_FIXED         )
      .value("mjCAMLIGHT_TRACK"       , mjtCamLight              ::mjCAMLIGHT_TRACK         )
      .value("mjCAMLIGHT_TRACKCOM"    , mjtCamLight              ::mjCAMLIGHT_TRACKCOM      )
      .value("mjCAMLIGHT_TARGETBODY"  , mjtCamLight              ::mjCAMLIGHT_TARGETBODY    )
      .value("mjCAMLIGHT_TARGETBODYCOM", mjtCamLight              ::mjCAMLIGHT_TARGETBODYCOM )
  ;
  enum_<mjtTexture>("mjtTexture")
      .value("mjTEXTURE_2D"           , mjtTexture               ::mjTEXTURE_2D             )
      .value("mjTEXTURE_CUBE"         , mjtTexture               ::mjTEXTURE_CUBE           )
      .value("mjTEXTURE_SKYBOX"       , mjtTexture               ::mjTEXTURE_SKYBOX         )
  ;
  enum_<mjtIntegrator>("mjtIntegrator")
      .value("mjINT_EULER"            , mjtIntegrator            ::mjINT_EULER              )
      .value("mjINT_RK4"              , mjtIntegrator            ::mjINT_RK4                )
      .value("mjINT_IMPLICIT"         , mjtIntegrator            ::mjINT_IMPLICIT           )
      .value("mjINT_IMPLICITFAST"     , mjtIntegrator            ::mjINT_IMPLICITFAST       )
  ;
  enum_<mjtCone>("mjtCone")
      .value("mjCONE_PYRAMIDAL"       , mjtCone                  ::mjCONE_PYRAMIDAL         )
      .value("mjCONE_ELLIPTIC"        , mjtCone                  ::mjCONE_ELLIPTIC          )
  ;
  enum_<mjtJacobian>("mjtJacobian")
      .value("mjJAC_DENSE"            , mjtJacobian              ::mjJAC_DENSE              )
      .value("mjJAC_SPARSE"           , mjtJacobian              ::mjJAC_SPARSE             )
      .value("mjJAC_AUTO"             , mjtJacobian              ::mjJAC_AUTO               )
  ;
  enum_<mjtSolver>("mjtSolver")
      .value("mjSOL_PGS"              , mjtSolver                ::mjSOL_PGS                )
      .value("mjSOL_CG"               , mjtSolver                ::mjSOL_CG                 )
      .value("mjSOL_NEWTON"           , mjtSolver                ::mjSOL_NEWTON             )
  ;
  enum_<mjtEq>("mjtEq")
      .value("mjEQ_CONNECT"           , mjtEq                    ::mjEQ_CONNECT             )
      .value("mjEQ_WELD"              , mjtEq                    ::mjEQ_WELD                )
      .value("mjEQ_JOINT"             , mjtEq                    ::mjEQ_JOINT               )
      .value("mjEQ_TENDON"            , mjtEq                    ::mjEQ_TENDON              )
      .value("mjEQ_FLEX"              , mjtEq                    ::mjEQ_FLEX                )
      .value("mjEQ_DISTANCE"          , mjtEq                    ::mjEQ_DISTANCE            )
  ;
  enum_<mjtWrap>("mjtWrap")
      .value("mjWRAP_NONE"            , mjtWrap                  ::mjWRAP_NONE              )
      .value("mjWRAP_JOINT"           , mjtWrap                  ::mjWRAP_JOINT             )
      .value("mjWRAP_PULLEY"          , mjtWrap                  ::mjWRAP_PULLEY            )
      .value("mjWRAP_SITE"            , mjtWrap                  ::mjWRAP_SITE              )
      .value("mjWRAP_SPHERE"          , mjtWrap                  ::mjWRAP_SPHERE            )
      .value("mjWRAP_CYLINDER"        , mjtWrap                  ::mjWRAP_CYLINDER          )
  ;
  enum_<mjtTrn>("mjtTrn")
      .value("mjTRN_JOINT"            , mjtTrn                   ::mjTRN_JOINT              )
      .value("mjTRN_JOINTINPARENT"    , mjtTrn                   ::mjTRN_JOINTINPARENT      )
      .value("mjTRN_SLIDERCRANK"      , mjtTrn                   ::mjTRN_SLIDERCRANK        )
      .value("mjTRN_TENDON"           , mjtTrn                   ::mjTRN_TENDON             )
      .value("mjTRN_SITE"             , mjtTrn                   ::mjTRN_SITE               )
      .value("mjTRN_BODY"             , mjtTrn                   ::mjTRN_BODY               )
      .value("mjTRN_UNDEFINED"        , mjtTrn                   ::mjTRN_UNDEFINED          )
  ;
  enum_<mjtDyn>("mjtDyn")
      .value("mjDYN_NONE"             , mjtDyn                   ::mjDYN_NONE               )
      .value("mjDYN_INTEGRATOR"       , mjtDyn                   ::mjDYN_INTEGRATOR         )
      .value("mjDYN_FILTER"           , mjtDyn                   ::mjDYN_FILTER             )
      .value("mjDYN_FILTEREXACT"      , mjtDyn                   ::mjDYN_FILTEREXACT        )
      .value("mjDYN_MUSCLE"           , mjtDyn                   ::mjDYN_MUSCLE             )
      .value("mjDYN_USER"             , mjtDyn                   ::mjDYN_USER               )
  ;
  enum_<mjtGain>("mjtGain")
      .value("mjGAIN_FIXED"           , mjtGain                  ::mjGAIN_FIXED             )
      .value("mjGAIN_AFFINE"          , mjtGain                  ::mjGAIN_AFFINE            )
      .value("mjGAIN_MUSCLE"          , mjtGain                  ::mjGAIN_MUSCLE            )
      .value("mjGAIN_USER"            , mjtGain                  ::mjGAIN_USER              )
  ;
  enum_<mjtBias>("mjtBias")
      .value("mjBIAS_NONE"            , mjtBias                  ::mjBIAS_NONE              )
      .value("mjBIAS_AFFINE"          , mjtBias                  ::mjBIAS_AFFINE            )
      .value("mjBIAS_MUSCLE"          , mjtBias                  ::mjBIAS_MUSCLE            )
      .value("mjBIAS_USER"            , mjtBias                  ::mjBIAS_USER              )
  ;
  enum_<mjtObj>("mjtObj")
      .value("mjOBJ_UNKNOWN"          , mjtObj                   ::mjOBJ_UNKNOWN            )
      .value("mjOBJ_BODY"             , mjtObj                   ::mjOBJ_BODY               )
      .value("mjOBJ_XBODY"            , mjtObj                   ::mjOBJ_XBODY              )
      .value("mjOBJ_JOINT"            , mjtObj                   ::mjOBJ_JOINT              )
      .value("mjOBJ_DOF"              , mjtObj                   ::mjOBJ_DOF                )
      .value("mjOBJ_GEOM"             , mjtObj                   ::mjOBJ_GEOM               )
      .value("mjOBJ_SITE"             , mjtObj                   ::mjOBJ_SITE               )
      .value("mjOBJ_CAMERA"           , mjtObj                   ::mjOBJ_CAMERA             )
      .value("mjOBJ_LIGHT"            , mjtObj                   ::mjOBJ_LIGHT              )
      .value("mjOBJ_FLEX"             , mjtObj                   ::mjOBJ_FLEX               )
      .value("mjOBJ_MESH"             , mjtObj                   ::mjOBJ_MESH               )
      .value("mjOBJ_SKIN"             , mjtObj                   ::mjOBJ_SKIN               )
      .value("mjOBJ_HFIELD"           , mjtObj                   ::mjOBJ_HFIELD             )
      .value("mjOBJ_TEXTURE"          , mjtObj                   ::mjOBJ_TEXTURE            )
      .value("mjOBJ_MATERIAL"         , mjtObj                   ::mjOBJ_MATERIAL           )
      .value("mjOBJ_PAIR"             , mjtObj                   ::mjOBJ_PAIR               )
      .value("mjOBJ_EXCLUDE"          , mjtObj                   ::mjOBJ_EXCLUDE            )
      .value("mjOBJ_EQUALITY"         , mjtObj                   ::mjOBJ_EQUALITY           )
      .value("mjOBJ_TENDON"           , mjtObj                   ::mjOBJ_TENDON             )
      .value("mjOBJ_ACTUATOR"         , mjtObj                   ::mjOBJ_ACTUATOR           )
      .value("mjOBJ_SENSOR"           , mjtObj                   ::mjOBJ_SENSOR             )
      .value("mjOBJ_NUMERIC"          , mjtObj                   ::mjOBJ_NUMERIC            )
      .value("mjOBJ_TEXT"             , mjtObj                   ::mjOBJ_TEXT               )
      .value("mjOBJ_TUPLE"            , mjtObj                   ::mjOBJ_TUPLE              )
      .value("mjOBJ_KEY"              , mjtObj                   ::mjOBJ_KEY                )
      .value("mjOBJ_PLUGIN"           , mjtObj                   ::mjOBJ_PLUGIN             )
      .value("mjNOBJECT"              , mjtObj                   ::mjNOBJECT                )
  ;
  enum_<mjtConstraint>("mjtConstraint")
      .value("mjCNSTR_EQUALITY"       , mjtConstraint            ::mjCNSTR_EQUALITY         )
      .value("mjCNSTR_FRICTION_DOF"   , mjtConstraint            ::mjCNSTR_FRICTION_DOF     )
      .value("mjCNSTR_FRICTION_TENDON", mjtConstraint            ::mjCNSTR_FRICTION_TENDON  )
      .value("mjCNSTR_LIMIT_JOINT"    , mjtConstraint            ::mjCNSTR_LIMIT_JOINT      )
      .value("mjCNSTR_LIMIT_TENDON"   , mjtConstraint            ::mjCNSTR_LIMIT_TENDON     )
      .value("mjCNSTR_CONTACT_FRICTIONLESS", mjtConstraint            ::mjCNSTR_CONTACT_FRICTIONLESS)
      .value("mjCNSTR_CONTACT_PYRAMIDAL", mjtConstraint            ::mjCNSTR_CONTACT_PYRAMIDAL)
      .value("mjCNSTR_CONTACT_ELLIPTIC", mjtConstraint            ::mjCNSTR_CONTACT_ELLIPTIC )
  ;
  enum_<mjtConstraintState>("mjtConstraintState")
      .value("mjCNSTRSTATE_SATISFIED" , mjtConstraintState       ::mjCNSTRSTATE_SATISFIED   )
      .value("mjCNSTRSTATE_QUADRATIC" , mjtConstraintState       ::mjCNSTRSTATE_QUADRATIC   )
      .value("mjCNSTRSTATE_LINEARNEG" , mjtConstraintState       ::mjCNSTRSTATE_LINEARNEG   )
      .value("mjCNSTRSTATE_LINEARPOS" , mjtConstraintState       ::mjCNSTRSTATE_LINEARPOS   )
      .value("mjCNSTRSTATE_CONE"      , mjtConstraintState       ::mjCNSTRSTATE_CONE        )
  ;
  enum_<mjtSensor>("mjtSensor")
      .value("mjSENS_TOUCH"           , mjtSensor                ::mjSENS_TOUCH             )
      .value("mjSENS_ACCELEROMETER"   , mjtSensor                ::mjSENS_ACCELEROMETER     )
      .value("mjSENS_VELOCIMETER"     , mjtSensor                ::mjSENS_VELOCIMETER       )
      .value("mjSENS_GYRO"            , mjtSensor                ::mjSENS_GYRO              )
      .value("mjSENS_FORCE"           , mjtSensor                ::mjSENS_FORCE             )
      .value("mjSENS_TORQUE"          , mjtSensor                ::mjSENS_TORQUE            )
      .value("mjSENS_MAGNETOMETER"    , mjtSensor                ::mjSENS_MAGNETOMETER      )
      .value("mjSENS_RANGEFINDER"     , mjtSensor                ::mjSENS_RANGEFINDER       )
      .value("mjSENS_CAMPROJECTION"   , mjtSensor                ::mjSENS_CAMPROJECTION     )
      .value("mjSENS_JOINTPOS"        , mjtSensor                ::mjSENS_JOINTPOS          )
      .value("mjSENS_JOINTVEL"        , mjtSensor                ::mjSENS_JOINTVEL          )
      .value("mjSENS_TENDONPOS"       , mjtSensor                ::mjSENS_TENDONPOS         )
      .value("mjSENS_TENDONVEL"       , mjtSensor                ::mjSENS_TENDONVEL         )
      .value("mjSENS_ACTUATORPOS"     , mjtSensor                ::mjSENS_ACTUATORPOS       )
      .value("mjSENS_ACTUATORVEL"     , mjtSensor                ::mjSENS_ACTUATORVEL       )
      .value("mjSENS_ACTUATORFRC"     , mjtSensor                ::mjSENS_ACTUATORFRC       )
      .value("mjSENS_JOINTACTFRC"     , mjtSensor                ::mjSENS_JOINTACTFRC       )
      .value("mjSENS_BALLQUAT"        , mjtSensor                ::mjSENS_BALLQUAT          )
      .value("mjSENS_BALLANGVEL"      , mjtSensor                ::mjSENS_BALLANGVEL        )
      .value("mjSENS_JOINTLIMITPOS"   , mjtSensor                ::mjSENS_JOINTLIMITPOS     )
      .value("mjSENS_JOINTLIMITVEL"   , mjtSensor                ::mjSENS_JOINTLIMITVEL     )
      .value("mjSENS_JOINTLIMITFRC"   , mjtSensor                ::mjSENS_JOINTLIMITFRC     )
      .value("mjSENS_TENDONLIMITPOS"  , mjtSensor                ::mjSENS_TENDONLIMITPOS    )
      .value("mjSENS_TENDONLIMITVEL"  , mjtSensor                ::mjSENS_TENDONLIMITVEL    )
      .value("mjSENS_TENDONLIMITFRC"  , mjtSensor                ::mjSENS_TENDONLIMITFRC    )
      .value("mjSENS_FRAMEPOS"        , mjtSensor                ::mjSENS_FRAMEPOS          )
      .value("mjSENS_FRAMEQUAT"       , mjtSensor                ::mjSENS_FRAMEQUAT         )
      .value("mjSENS_FRAMEXAXIS"      , mjtSensor                ::mjSENS_FRAMEXAXIS        )
      .value("mjSENS_FRAMEYAXIS"      , mjtSensor                ::mjSENS_FRAMEYAXIS        )
      .value("mjSENS_FRAMEZAXIS"      , mjtSensor                ::mjSENS_FRAMEZAXIS        )
      .value("mjSENS_FRAMELINVEL"     , mjtSensor                ::mjSENS_FRAMELINVEL       )
      .value("mjSENS_FRAMEANGVEL"     , mjtSensor                ::mjSENS_FRAMEANGVEL       )
      .value("mjSENS_FRAMELINACC"     , mjtSensor                ::mjSENS_FRAMELINACC       )
      .value("mjSENS_FRAMEANGACC"     , mjtSensor                ::mjSENS_FRAMEANGACC       )
      .value("mjSENS_SUBTREECOM"      , mjtSensor                ::mjSENS_SUBTREECOM        )
      .value("mjSENS_SUBTREELINVEL"   , mjtSensor                ::mjSENS_SUBTREELINVEL     )
      .value("mjSENS_SUBTREEANGMOM"   , mjtSensor                ::mjSENS_SUBTREEANGMOM     )
      .value("mjSENS_CLOCK"           , mjtSensor                ::mjSENS_CLOCK             )
      .value("mjSENS_PLUGIN"          , mjtSensor                ::mjSENS_PLUGIN            )
      .value("mjSENS_USER"            , mjtSensor                ::mjSENS_USER              )
  ;
  enum_<mjtStage>("mjtStage")
      .value("mjSTAGE_NONE"           , mjtStage                 ::mjSTAGE_NONE             )
      .value("mjSTAGE_POS"            , mjtStage                 ::mjSTAGE_POS              )
      .value("mjSTAGE_VEL"            , mjtStage                 ::mjSTAGE_VEL              )
      .value("mjSTAGE_ACC"            , mjtStage                 ::mjSTAGE_ACC              )
  ;
  enum_<mjtDataType>("mjtDataType")
      .value("mjDATATYPE_REAL"        , mjtDataType              ::mjDATATYPE_REAL          )
      .value("mjDATATYPE_POSITIVE"    , mjtDataType              ::mjDATATYPE_POSITIVE      )
      .value("mjDATATYPE_AXIS"        , mjtDataType              ::mjDATATYPE_AXIS          )
      .value("mjDATATYPE_QUATERNION"  , mjtDataType              ::mjDATATYPE_QUATERNION    )
  ;
  enum_<mjtLRMode>("mjtLRMode")
      .value("mjLRMODE_NONE"          , mjtLRMode                ::mjLRMODE_NONE            )
      .value("mjLRMODE_MUSCLE"        , mjtLRMode                ::mjLRMODE_MUSCLE          )
      .value("mjLRMODE_MUSCLEUSER"    , mjtLRMode                ::mjLRMODE_MUSCLEUSER      )
      .value("mjLRMODE_ALL"           , mjtLRMode                ::mjLRMODE_ALL             )
  ;
  enum_<mjtFlexSelf>("mjtFlexSelf")
      .value("mjFLEXSELF_NONE"        , mjtFlexSelf              ::mjFLEXSELF_NONE          )
      .value("mjFLEXSELF_NARROW"      , mjtFlexSelf              ::mjFLEXSELF_NARROW        )
      .value("mjFLEXSELF_BVH"         , mjtFlexSelf              ::mjFLEXSELF_BVH           )
      .value("mjFLEXSELF_SAP"         , mjtFlexSelf              ::mjFLEXSELF_SAP           )
      .value("mjFLEXSELF_AUTO"        , mjtFlexSelf              ::mjFLEXSELF_AUTO          )
  ;


  class_<Model>("Model")
      .constructor<>(&Model::load_from_xml)
      .class_function("load_from_xml", &Model::load_from_xml)
      .class_function("load_from_mjb", &Model::load_from_mjb)
      .function("ptr", &Model::ptr, allow_raw_pointers())
      .function("free"            , &Model::free        )
      .function("getVal"          , &Model::getVal      )
      .function("getOptions"      , &Model::getOptions  )
      // MJMODEL_BINDINGS
      .property("nv ", &Model::nv )              
      .property("nu", &Model::nu )              
      .property("na", &Model::na )              
      .property("nbody", &Model::nbody )           
      .property("nbvh", &Model::nbvh )            
      .property("nbvhstatic", &Model::nbvhstatic )      
      .property("njnt", &Model::njnt )            
      .property("ngeom ", &Model::ngeom )           
      .property("nsite ", &Model::nsite )           
      .property("ncam", &Model::ncam )            
      .property("nlight", &Model::nlight )          
      .property("nflex", &Model::nflex )           
      .property("nflexvert", &Model::nflexvert )       
      .property("nflextexcoord", &Model::nflextexcoord )   
      .property("nmesh ", &Model::nmesh )           
      .property("nskin", &Model::nskin )           
      .property("nskinvert", &Model::nskinvert )       
      .property("nskinface", &Model::nskinface )       
      .property("nskinbone", &Model::nskinbone )       
      .property("nskinbonevert", &Model::nskinbonevert )   
      .property("nmat ", &Model::nmat )            
      .property("neq ", &Model::neq )             
      .property("ntendon", &Model::ntendon )         
      .property("nwrap", &Model::nwrap )           
      .property("nsensor", &Model::nsensor )         
      .property("nnames", &Model::nnames )          
      .property("npaths", &Model::npaths )          
      .property("ntree", &Model::ntree )           
      .property("nsensordata", &Model::nsensordata )     
      .property("body_parentid", &Model::body_parentid)
      .property("body_rootid", &Model::body_rootid)
      .property("body_weldid", &Model::body_weldid)
      .property("body_mocapid", &Model::body_mocapid)
      .property("body_jntnum", &Model::body_jntnum)
      .property("body_jntadr", &Model::body_jntadr)
      .property("body_dofnum", &Model::body_dofnum)
      .property("body_dofadr", &Model::body_dofadr)
      .property("body_geomnum", &Model::body_geomnum)
      .property("body_geomadr", &Model::body_geomadr)
      .property("body_iquat", &Model::body_iquat)
      .property("body_mass", &Model::body_mass)
      .property("body_inertia", &Model::body_inertia)
      .property("body_bvhadr", &Model::body_bvhadr)
      .property("body_bvhnum", &Model::body_bvhnum)
      .property("bvh_depth", &Model::bvh_depth)
      .property("bvh_child", &Model::bvh_child)
      .property("bvh_nodeid", &Model::bvh_nodeid)
      .property("bvh_aabb", &Model::bvh_aabb)
      .property("jnt_type", &Model::jnt_type)
      .property("jnt_bodyid", &Model::jnt_bodyid)
      .property("jnt_group", &Model::jnt_group)
      .property("geom_type", &Model::geom_type)
      .property("geom_contype", &Model::geom_contype)
      .property("geom_conaffinity", &Model::geom_conaffinity)
      .property("geom_bodyid", &Model::geom_bodyid)
      .property("geom_dataid", &Model::geom_dataid)
      .property("geom_matid", &Model::geom_matid)
      .property("geom_group", &Model::geom_group)
      .property("geom_size", &Model::geom_size)
      .property("geom_aabb", &Model::geom_aabb)
      .property("geom_rbound", &Model::geom_rbound)
      .property("geom_rgba", &Model::geom_rgba)
      .property("site_type", &Model::site_type)
      .property("site_bodyid", &Model::site_bodyid)
      .property("site_matid", &Model::site_matid)
      .property("site_group", &Model::site_group)
      .property("site_size", &Model::site_size)
      .property("site_rgba", &Model::site_rgba)
      .property("cam_sensorsize", &Model::cam_sensorsize)
      .property("cam_intrinsic", &Model::cam_intrinsic)
      .property("cam_fovy", &Model::cam_fovy)
      .property("cam_ipd", &Model::cam_ipd)
      .property("light_directional", &Model::light_directional)
      .property("light_castshadow", &Model::light_castshadow)
      .property("light_active", &Model::light_active)
      .property("light_attenuation", &Model::light_attenuation)
      .property("light_cutoff", &Model::light_cutoff)
      .property("light_exponent", &Model::light_exponent)
      .property("light_ambient", &Model::light_ambient)
      .property("light_diffuse", &Model::light_diffuse)
      .property("light_specular", &Model::light_specular)
      .property("flex_dim", &Model::flex_dim)
      .property("flex_matid", &Model::flex_matid)
      .property("flex_group", &Model::flex_group)
      .property("flex_vertadr", &Model::flex_vertadr)
      .property("flex_vertnum", &Model::flex_vertnum)
      .property("flex_elemadr", &Model::flex_elemadr)
      .property("flex_elemnum", &Model::flex_elemnum)
      .property("flex_elemdataadr", &Model::flex_elemdataadr)
      .property("flex_shellnum", &Model::flex_shellnum)
      .property("flex_shelldataadr", &Model::flex_shelldataadr)
      .property("flex_texcoordadr", &Model::flex_texcoordadr)
      .property("flex_elem", &Model::flex_elem)
      .property("flex_elemlayer", &Model::flex_elemlayer)
      .property("flex_shell", &Model::flex_shell)
      .property("flex_radius", &Model::flex_radius)
      .property("flex_flatskin", &Model::flex_flatskin)
      .property("flex_bvhadr", &Model::flex_bvhadr)
      .property("flex_bvhnum", &Model::flex_bvhnum)
      .property("flex_rgba", &Model::flex_rgba)
      .property("mesh_texcoordadr", &Model::mesh_texcoordadr)
      .property("mesh_bvhadr", &Model::mesh_bvhadr)
      .property("mesh_bvhnum", &Model::mesh_bvhnum)
      .property("mesh_graphadr", &Model::mesh_graphadr)
      .property("mesh_pathadr", &Model::mesh_pathadr)
      .property("skin_matid", &Model::skin_matid)
      .property("skin_group", &Model::skin_group)
      .property("skin_rgba", &Model::skin_rgba)
      .property("skin_inflate", &Model::skin_inflate)
      .property("skin_vertadr", &Model::skin_vertadr)
      .property("skin_vertnum", &Model::skin_vertnum)
      .property("skin_texcoordadr", &Model::skin_texcoordadr)
      .property("skin_faceadr", &Model::skin_faceadr)
      .property("skin_facenum", &Model::skin_facenum)
      .property("skin_boneadr", &Model::skin_boneadr)
      .property("skin_bonenum", &Model::skin_bonenum)
      .property("skin_vert", &Model::skin_vert)
      .property("skin_face", &Model::skin_face)
      .property("skin_bonevertadr", &Model::skin_bonevertadr)
      .property("skin_bonevertnum", &Model::skin_bonevertnum)
      .property("skin_bonebindpos", &Model::skin_bonebindpos)
      .property("skin_bonebindquat", &Model::skin_bonebindquat)
      .property("skin_bonebodyid", &Model::skin_bonebodyid)
      .property("skin_bonevertid", &Model::skin_bonevertid)
      .property("skin_bonevertweight", &Model::skin_bonevertweight)
      .property("skin_pathadr", &Model::skin_pathadr)
      .property("hfield_pathadr", &Model::hfield_pathadr)
      .property("tex_pathadr", &Model::tex_pathadr)
      .property("mat_texid", &Model::mat_texid)
      .property("mat_texuniform", &Model::mat_texuniform)
      .property("mat_texrepeat", &Model::mat_texrepeat)
      .property("mat_emission", &Model::mat_emission)
      .property("mat_specular", &Model::mat_specular)
      .property("mat_shininess", &Model::mat_shininess)
      .property("mat_reflectance", &Model::mat_reflectance)
      .property("mat_rgba", &Model::mat_rgba)
      .property("eq_type", &Model::eq_type)
      .property("eq_obj1id", &Model::eq_obj1id)
      .property("eq_obj2id", &Model::eq_obj2id)
      .property("eq_data", &Model::eq_data)
      .property("tendon_num", &Model::tendon_num)
      .property("tendon_matid", &Model::tendon_matid)
      .property("tendon_group", &Model::tendon_group)
      .property("tendon_limited", &Model::tendon_limited)
      .property("tendon_width", &Model::tendon_width)
      .property("tendon_range", &Model::tendon_range)
      .property("tendon_stiffness", &Model::tendon_stiffness)
      .property("tendon_damping", &Model::tendon_damping)
      .property("tendon_frictionloss", &Model::tendon_frictionloss)
      .property("tendon_lengthspring", &Model::tendon_lengthspring)
      .property("tendon_rgba", &Model::tendon_rgba)
      .property("actuator_trntype", &Model::actuator_trntype)
      .property("actuator_dyntype", &Model::actuator_dyntype)
      .property("actuator_trnid", &Model::actuator_trnid)
      .property("actuator_actadr", &Model::actuator_actadr)
      .property("actuator_actnum", &Model::actuator_actnum)
      .property("actuator_group", &Model::actuator_group)
      .property("actuator_ctrllimited", &Model::actuator_ctrllimited)
      .property("actuator_actlimited", &Model::actuator_actlimited)
      .property("actuator_ctrlrange", &Model::actuator_ctrlrange)
      .property("actuator_actrange", &Model::actuator_actrange)
      .property("actuator_cranklength", &Model::actuator_cranklength)
      .property("sensor_type", &Model::sensor_type)
      .property("sensor_objid", &Model::sensor_objid)
      .property("sensor_adr", &Model::sensor_adr)
      .property("name_bodyadr", &Model::name_bodyadr)
      .property("name_jntadr", &Model::name_jntadr)
      .property("name_geomadr", &Model::name_geomadr)
      .property("name_siteadr", &Model::name_siteadr)
      .property("name_camadr", &Model::name_camadr)
      .property("name_lightadr", &Model::name_lightadr)
      .property("name_eqadr", &Model::name_eqadr)
      .property("name_tendonadr", &Model::name_tendonadr)
      .property("name_actuatoradr", &Model::name_actuatoradr)
      .property("names", &Model::names)
      .property("paths", &Model::paths)
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
      .property("qpos"                  , &Simulation::qpos)                 )
      .property("qvel"                  , &Simulation::qvel)   
      .property("act", &Simulation::act)
      .property("ctrl", &Simulation::ctrl)
      .property("xfrc_applied", &Simulation::xfrc_applied)
      .property("eq_active", &Simulation::eq_active)
      .property("sensordata", &Simulation::sensordata)
      .property("xpos", &Simulation::xpos)
      .property("xquat", &Simulation::xquat)
      .property("xmat", &Simulation::xmat)
      .property("xipos", &Simulation::xipos)
      .property("ximat", &Simulation::ximat)
      .property("xanchor", &Simulation::xanchor)
      .property("xaxis", &Simulation::xaxis)
      .property("geom_xpos", &Simulation::geom_xpos)
      .property("geom_xmat", &Simulation::geom_xmat)
      .property("site_xpos", &Simulation::site_xpos)
      .property("site_xmat", &Simulation::site_xmat)
      .property("cam_xpos", &Simulation::cam_xpos)
      .property("cam_xmat", &Simulation::cam_xmat)
      .property("light_xpos", &Simulation::light_xpos)
      .property("light_xdir", &Simulation::light_xdir)
      .property("subtree_com", &Simulation::subtree_com)
      .property("flexvert_xpos", &Simulation::flexvert_xpos)
      .property("ten_wrapadr", &Simulation::ten_wrapadr)
      .property("ten_wrapnum", &Simulation::ten_wrapnum)
      .property("ten_length", &Simulation::ten_length)
      .property("wrap_obj", &Simulation::wrap_obj)
      .property("wrap_xpos", &Simulation::wrap_xpos)
      .property("bvh_aabb_dyn", &Simulation::bvh_aabb_dyn)
      .property("bvh_active", &Simulation::bvh_active)
      .property("contact", &Simulation::contact)
      .property("efc_type", &Simulation::efc_type)
      .property("efc_id", &Simulation::efc_id)
      .property("efc_J_rownnz", &Simulation::efc_J_rownnz)
      .property("efc_J_rowadr", &Simulation::efc_J_rowadr)
      .property("efc_J_rowsuper", &Simulation::efc_J_rowsuper)
      .property("efc_J_colind", &Simulation::efc_J_colind)
      .property("efc_JT_rownnz", &Simulation::efc_JT_rownnz)
      .property("efc_JT_rowadr", &Simulation::efc_JT_rowadr)
      .property("efc_JT_rowsuper", &Simulation::efc_JT_rowsuper)
      .property("efc_JT_colind", &Simulation::efc_JT_colind)
      .property("efc_J", &Simulation::efc_J)
      .property("efc_JT", &Simulation::efc_JT)
      .property("efc_pos", &Simulation::efc_pos)
      .property("efc_margin", &Simulation::efc_margin)
      .property("efc_frictionloss", &Simulation::efc_frictionloss)
      .property("efc_diagApprox", &Simulation::efc_diagApprox)
      .property("efc_KBIP", &Simulation::efc_KBIP)
      .property("efc_D", &Simulation::efc_D)
      .property("efc_R", &Simulation::efc_R)
      .property("tendon_efcadr", &Simulation::tendon_efcadr)
      .property("efc_vel", &Simulation::efc_vel)
      .property("efc_aref", &Simulation::efc_aref)
      .property("efc_b", &Simulation::efc_b)
      .property("efc_force", &Simulation::efc_force)
      .property("efc_state", &Simulation::efc_state)
      .property("efc_AR_rownnz", &Simulation::efc_AR_rownnz)
      .property("efc_AR_rowadr", &Simulation::efc_AR_rowadr)
      .property("efc_AR_colind", &Simulation::efc_AR_colind)
      .property("efc_AR", &Simulation::efc_AR)
      .property("dof_island", &Simulation::dof_island)
      .property("island_dofnum", &Simulation::island_dofnum)
      .property("island_dofadr", &Simulation::island_dofadr)
      .property("island_dofind", &Simulation::island_dofind)
      .property("dof_islandind", &Simulation::dof_islandind)
      .property("efc_island", &Simulation::efc_island)
      .property("island_efcnum", &Simulation::island_efcnum)
      .property("island_efcadr", &Simulation::island_efcadr)
      .property("island_efcind", &Simulation::island_efcind)
      .property("narena", &Simulation::narena)
      .property("nbuffer", &Simulation::nbuffer)
      .property("nplugin", &Simulation::nplugin)
      .property("pstack", &Simulation::pstack)
      .property("pbase ", &Simulation::pbase )
      .property("parena", &Simulation::parena)
      .property("maxuse_stack", &Simulation::maxuse_stack)
      .property("maxuse_arena", &Simulation::maxuse_arena)
      .property("maxuse_con  ", &Simulation::maxuse_con  )
      .property("maxuse_efc  ", &Simulation::maxuse_efc  )
      .property("solver_nisland", &Simulation::solver_nisland)
      .property("ne    ", &Simulation::ne    )
      .property("nf    ", &Simulation::nf    )
      .property("nl    ", &Simulation::nl    )
      .property("nefc  ", &Simulation::nefc  )
      .property("nnzJ  ", &Simulation::nnzJ  )
      .property("ncon  ", &Simulation::ncon  )
      .property("nisland     ", &Simulation::nisland     )
      .property("time  ", &Simulation::time  )
      .property("threadpool         ", &Simulation::threadpool         )
      .property("maxuse_threadstack", &Simulation::maxuse_threadstack)
      .property("warning", &Simulation::warning)
      .property("timer", &Simulation::timer)
      .property("solver", &Simulation::solver)
      .property("solver_niter", &Simulation::solver_niter)
      .property("solver_nnz", &Simulation::solver_nnz)
      .property("solver_fwdinv", &Simulation::solver_fwdinv)
      .property("energy", &Simulation::energy)
      .function("freeLastXML"           , &Simulation::freeLastXML           )
      .function("step"                  , &Simulation::step                  )
      .function("step1"                 , &Simulation::step1                 )
      .function("step2"                 , &Simulation::step2                 )
      .function("forward"               , &Simulation::forward               )
      .function("inverse"               , &Simulation::inverse               )
      .function("forwardSkip"           , &Simulation::forwardSkip           )
      .function("inverseSkip"           , &Simulation::inverseSkip           )
      .function("defaultSolRefImp"      , &Simulation::defaultSolRefImp      , allow_raw_pointers())
      .function("sizeModel"             , &Simulation::sizeModel             )
      .function("resetData"             , &Simulation::resetData             )
      .function("resetDataDebug"        , &Simulation::resetDataDebug        )
      .function("resetDataKeyframe"     , &Simulation::resetDataKeyframe     )
      .function("markStack"             , &Simulation::markStack             )
      .function("freeStack"             , &Simulation::freeStack             )
      .function("deleteData"            , &Simulation::deleteData            )
      .function("resetCallbacks"        , &Simulation::resetCallbacks        )
      .function("printFormattedModel"   , &Simulation::printFormattedModel   )
      .function("printModel"            , &Simulation::printModel            )
      .function("printFormattedData"    , &Simulation::printFormattedData    )
      .function("printData"             , &Simulation::printData             )
      .function("_printMat"             , &Simulation::_printMat             , allow_raw_pointers())
      .function("fwdPosition"           , &Simulation::fwdPosition           )
      .function("fwdVelocity"           , &Simulation::fwdVelocity           )
      .function("fwdActuation"          , &Simulation::fwdActuation          )
      .function("fwdAcceleration"       , &Simulation::fwdAcceleration       )
      .function("fwdConstraint"         , &Simulation::fwdConstraint         )
      .function("Euler"                 , &Simulation::Euler                 )
      .function("RungeKutta"            , &Simulation::RungeKutta            )
      .function("implicit"              , &Simulation::implicit              )
      .function("invPosition"           , &Simulation::invPosition           )
      .function("invVelocity"           , &Simulation::invVelocity           )
      .function("invConstraint"         , &Simulation::invConstraint         )
      .function("compareFwdInv"         , &Simulation::compareFwdInv         )
      .function("sensorPos"             , &Simulation::sensorPos             )
      .function("sensorVel"             , &Simulation::sensorVel             )
      .function("sensorAcc"             , &Simulation::sensorAcc             )
      .function("energyPos"             , &Simulation::energyPos             )
      .function("energyVel"             , &Simulation::energyVel             )
      .function("checkPos"              , &Simulation::checkPos              )
      .function("checkVel"              , &Simulation::checkVel              )
      .function("checkAcc"              , &Simulation::checkAcc              )
      .function("kinematics"            , &Simulation::kinematics            )
      .function("comPos"                , &Simulation::comPos                )
      .function("camlight"              , &Simulation::camlight              )
      .function("flex"                  , &Simulation::flex                  )
      .function("tendon"                , &Simulation::tendon                )
      .function("transmission"          , &Simulation::transmission          )
      .function("crbCalculate"          , &Simulation::crbCalculate          )
      .function("factorM"               , &Simulation::factorM               )
      .function("solveM"                , &Simulation::solveM                , allow_raw_pointers())
      .function("solveM2"               , &Simulation::solveM2               , allow_raw_pointers())
      .function("comVel"                , &Simulation::comVel                )
      .function("passive"               , &Simulation::passive               )
      .function("subtreeVel"            , &Simulation::subtreeVel            )
      .function("rne"                   , &Simulation::rne                   , allow_raw_pointers())
      .function("rnePostConstraint"     , &Simulation::rnePostConstraint     )
      .function("collision"             , &Simulation::collision             )
      .function("makeConstraint"        , &Simulation::makeConstraint        )
      .function("island"                , &Simulation::island                )
      .function("projectConstraint"     , &Simulation::projectConstraint     )
      .function("referenceConstraint"   , &Simulation::referenceConstraint   )
      .function("stateSize"             , &Simulation::stateSize             )
      .function("setState"              , &Simulation::setState              , allow_raw_pointers())
      .function("isPyramidal"           , &Simulation::isPyramidal           )
      .function("isSparse"              , &Simulation::isSparse              )
      .function("isDual"                , &Simulation::isDual                )
      .function("jacSubtreeCom"         , &Simulation::jacSubtreeCom         , allow_raw_pointers())
      .function("name2id"               , &Simulation::name2id               )
      .function("id2name"               , &Simulation::id2name               )
      .function("fullM"                 , &Simulation::fullM                 , allow_raw_pointers())
      .function("differentiatePos"      , &Simulation::differentiatePos      , allow_raw_pointers())
      .function("integratePos"          , &Simulation::integratePos          , allow_raw_pointers())
      .function("normalizeQuat"         , &Simulation::normalizeQuat         , allow_raw_pointers())
      .function("getTotalmass"          , &Simulation::getTotalmass          )
      .function("getPluginConfig"       , &Simulation::getPluginConfig       )
      .function("loadPluginLibrary"     , &Simulation::loadPluginLibrary     )
      .function("version"               , &Simulation::version               )
      .function("versionString"         , &Simulation::versionString         )
      .function("_rectangle"            , &Simulation::_rectangle            )
      .function("_finish"               , &Simulation::_finish               )
      .function("_getError"             , &Simulation::_getError             )
      .function("i_themeSpacing"        , &Simulation::i_themeSpacing        )
      .function("i_themeColor"          , &Simulation::i_themeColor          )
      .function("_error"                , &Simulation::_error                )
      .function("_error_i"              , &Simulation::_error_i              )
      .function("_error_s"              , &Simulation::_error_s              )
      .function("_warning"              , &Simulation::_warning              )
      .function("_warning_i"            , &Simulation::_warning_i            )
      .function("_warning_s"            , &Simulation::_warning_s            )
      .function("_clearHandlers"        , &Simulation::_clearHandlers        )
      .function("warning"               , &Simulation::warning               )
      .function("_writeLog"             , &Simulation::_writeLog             )
      .function("_zero"                 , &Simulation::_zero                 , allow_raw_pointers())
      .function("_fill"                 , &Simulation::_fill                 , allow_raw_pointers())
      .function("_copy"                 , &Simulation::_copy                 , allow_raw_pointers())
      .function("_sum"                  , &Simulation::_sum                  , allow_raw_pointers())
      .function("_L1"                   , &Simulation::_L1                   , allow_raw_pointers())
      .function("_scl"                  , &Simulation::_scl                  , allow_raw_pointers())
      .function("_add"                  , &Simulation::_add                  , allow_raw_pointers())
      .function("_sub"                  , &Simulation::_sub                  , allow_raw_pointers())
      .function("_addTo"                , &Simulation::_addTo                , allow_raw_pointers())
      .function("_subFrom"              , &Simulation::_subFrom              , allow_raw_pointers())
      .function("_addToScl"             , &Simulation::_addToScl             , allow_raw_pointers())
      .function("_addScl"               , &Simulation::_addScl               , allow_raw_pointers())
      .function("_normalize"            , &Simulation::_normalize            , allow_raw_pointers())
      .function("_norm"                 , &Simulation::_norm                 , allow_raw_pointers())
      .function("_dot"                  , &Simulation::_dot                  , allow_raw_pointers())
      .function("_mulMatVec"            , &Simulation::_mulMatVec            , allow_raw_pointers())
      .function("_mulMatTVec"           , &Simulation::_mulMatTVec           , allow_raw_pointers())
      .function("_mulVecMatVec"         , &Simulation::_mulVecMatVec         , allow_raw_pointers())
      .function("_transpose"            , &Simulation::_transpose            , allow_raw_pointers())
      .function("_symmetrize"           , &Simulation::_symmetrize           , allow_raw_pointers())
      .function("_eye"                  , &Simulation::_eye                  , allow_raw_pointers())
      .function("_mulMatMat"            , &Simulation::_mulMatMat            , allow_raw_pointers())
      .function("_mulMatMatT"           , &Simulation::_mulMatMatT           , allow_raw_pointers())
      .function("_mulMatTMat"           , &Simulation::_mulMatTMat           , allow_raw_pointers())
      .function("_sqrMatTD"             , &Simulation::_sqrMatTD             , allow_raw_pointers())
      .function("_cholFactor"           , &Simulation::_cholFactor           , allow_raw_pointers())
      .function("_cholSolve"            , &Simulation::_cholSolve            , allow_raw_pointers())
      .function("_cholUpdate"           , &Simulation::_cholUpdate           , allow_raw_pointers())
      .function("_cholFactorBand"       , &Simulation::_cholFactorBand       , allow_raw_pointers())
      .function("_cholSolveBand"        , &Simulation::_cholSolveBand        , allow_raw_pointers())
      .function("_band2Dense"           , &Simulation::_band2Dense           , allow_raw_pointers())
      .function("_dense2Band"           , &Simulation::_dense2Band           , allow_raw_pointers())
      .function("_bandMulMatVec"        , &Simulation::_bandMulMatVec        , allow_raw_pointers())
      .function("_bandDiag"             , &Simulation::_bandDiag             )
      .function("_encodePyramid"        , &Simulation::_encodePyramid        , allow_raw_pointers())
      .function("_decodePyramid"        , &Simulation::_decodePyramid        , allow_raw_pointers())
      .function("_springDamper"         , &Simulation::_springDamper         )
      .function("_min"                  , &Simulation::_min                  )
      .function("_max"                  , &Simulation::_max                  )
      .function("_clip"                 , &Simulation::_clip                 )
      .function("_sign"                 , &Simulation::_sign                 )
      .function("_round"                , &Simulation::_round                )
      .function("_type2Str"             , &Simulation::_type2Str             )
      .function("_str2Type"             , &Simulation::_str2Type             )
      .function("_writeNumBytes"        , &Simulation::_writeNumBytes        )
      .function("_warningText"          , &Simulation::_warningText          )
      .function("_isBad"                , &Simulation::_isBad                )
      .function("_isZero"               , &Simulation::_isZero               , allow_raw_pointers())
      .function("_standardNormal"       , &Simulation::_standardNormal       , allow_raw_pointers())
      .function("_insertionSort"        , &Simulation::_insertionSort        , allow_raw_pointers())
      .function("_Halton"               , &Simulation::_Halton               )
      .function("_sigmoid"              , &Simulation::_sigmoid              )
      .function("_transitionFD"         , &Simulation::_transitionFD         , allow_raw_pointers())
      .function("_inverseFD"            , &Simulation::_inverseFD            , allow_raw_pointers())
      .function("_pluginCount"          , &Simulation::_pluginCount          )
      .function("_resourceProviderCount" , &Simulation::_resourceProviderCount)
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
      .field("interval"    , &mjLROpt::interval)    // evaluation time interval (at the end)
      .field("tolrange"   , &mjLROpt::tolrange);  // convergence tolerance (relative to range)

  value_object<mjOption>("mjOption")
      .field("timestep"            , &mjOption::timestep)          // timestep
      .field("apirate"             , &mjOption::apirate)           // update rate for remote API (Hz)
      .field("impratio"            , &mjOption::impratio)          // ratio of friction-to-normal contact impedance
      .field("tolerance"           , &mjOption::tolerance)         // main solver tolerance
      .field("noslip_tolerance"    , &mjOption::noslip_tolerance)  // noslip solver tolerance
      .field("mpr_tolerance"       , &mjOption::mpr_tolerance)     // MPR solver tolerance
      //.field("gravity"           , &mjOption::gravity)           // gravitational acceleration
      //.field("wind"              , &mjOption::wind)              // wind (for lift, drag and viscosity)
      //.field("magnetic"          , &mjOption::magnetic)          // global magnetic flux
      .field("density"             , &mjOption::density)           // density of medium
      .field("viscosity"           , &mjOption::viscosity)         // viscosity of medium
      .field("o_margin"            , &mjOption::o_margin)          // margin
      //.field("o_solref"          , &mjOption::o_solref)          // solref
      //.field("o_solimp"          , &mjOption::o_solimp)          // solimp
      .field("integrator"          , &mjOption::integrator)        // integration mode (mjtIntegrator)
      .field("cone"                , &mjOption::cone)              // type of friction cone (mjtCone)
      .field("jacobian"            , &mjOption::jacobian)          // type of Jacobian (mjtJacobian)
      .field("solver"              , &mjOption::solver)            // solver algorithm (mjtSolver)
      .field("iterations"          , &mjOption::iterations)        // maximum number of main solver iterations
      .field("noslip_iterations"   , &mjOption::noslip_iterations) // maximum number of noslip solver iterations
      .field("mpr_iterations"      , &mjOption::mpr_iterations)    // maximum number of MPR solver iterations
      .field("disableflags"        , &mjOption::disableflags)      // bit flags for disabling standard features
      .field("enableflags"         , &mjOption::enableflags);      // bit flags for enabling optional features

  register_vector<mjContact>("vector<mjContact>");
}
