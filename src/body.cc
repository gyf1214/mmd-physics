#include "ext.hpp"
#include "mmd-physics/body.hpp"
#include <glm/gtc/quaternion.hpp>

using namespace std;
using namespace glm;

static const double gravity = 9.80665*20/1.58;
static const int maxSubstep = 10;
static const float frequency = 1.0f / 300.0f;

namespace mmd {
    namespace physics {
        inline btVector3 gl2bt(const vec3 &v) {
            return btVector3(v[0], v[1], v[2]);
        }

        inline vec3 bt2gl(const btVector3 &v) {
            return vec3(v[0], v[1], v[2]);
        }

        inline mat4 bt2gl(const btTransform &t) {
            const auto &T = t.getOrigin();
            const auto &M = t.getBasis();

            return mat4(
                M[0][0], M[1][0], M[2][0], 0.0f,
                M[0][1], M[1][1], M[2][1], 0.0f,
                M[0][2], M[1][2], M[2][2], 0.0f,
                T[0]   , T[1]   , T[2]   , 1.0f
            );
        }

        inline btTransform gl2bt(const mat4 &m) {
            btMatrix3x3 basis(
                m[0][0], m[1][0], m[2][0],
                m[0][1], m[1][1], m[2][1],
                m[0][2], m[1][2], m[2][2]
            );

            return btTransform(basis, btVector3(m[3][0], m[3][1], m[3][2]));
        }

        inline mat4 mmd2gl(const vec3 &pos, const vec3 &rot) {
            return translate(pos) * mat4_cast(quat(rot));
        }

        struct Rigid {
            const pmx::Rigid *base;
            btRigidBody *bt;
            btCollisionShape *shape;
            btMotionState *motion;

            mat4 W0, S0i, Ti;

            Rigid() : bt(NULL), shape(NULL), motion(NULL) {}

            void createRigid() {
                switch (base->shape) {
                    case pmx::Rigid::RigidBox:
                    shape = new btBoxShape(gl2bt(base->size / 2.0f));
                    break;
                    case pmx::Rigid::RigidSphere:
                    shape = new btSphereShape(base->size.x / 2.0f);
                    break;
                    case pmx::Rigid::RigidCapsule:
                    shape = new btCapsuleShape(base->size.x, base->size.y);
                    break;
                }

                btVector3 inert(0.0f, 0.0f, 0.0f);
                motion = new btDefaultMotionState(gl2bt(W0));
                bool kinematic = !base->mode;
                float mass = kinematic ? 0.0f : base->mass;
                if (mass > 0) shape->calculateLocalInertia(mass, inert);
                btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, inert);
                bt = new btRigidBody(info);
                bt->setRestitution(base->repulsion);
                bt->setFriction(base->friction);
                bt->setDamping(base->moveDamping, base->rotateDamping);
                if (kinematic) {
                    bt->setCollisionFlags(bt->getCollisionFlags() |
                                          btCollisionObject::CF_KINEMATIC_OBJECT);
                }
                bt->setActivationState(DISABLE_DEACTIVATION);
            }

            void load(const pmx::Model *model, int index) {
                base = &model->rigids[index];

                W0 = mmd2gl(base->position, base->rotation);
                if (base->bone >= 0) {
                    const auto &bone = model->bones[base->bone];
                    vec3 p = -bone.position;
                    if (bone.parent >= 0) p += model->bones[bone.parent].position;
                    Ti = translate(p);
                    S0i = inverse(W0) * translate(bone.position);
                } else {
                    Ti = mat4(1.0f);
                    S0i = inverse(W0);
                }

                createRigid();
            }

            void reset() {
                delete bt;
                delete motion;
                delete shape;
            }
        };

        Body::~Body() {}

        class BodyImp : public Body {
            const pmx::Model *model;
            vector<Rigid> rigids;
            vector<btTypedConstraint *> joints;
            Armature *armature;
            mat4 mMat;

            struct {
                btDiscreteDynamicsWorld *base;
                btCollisionDispatcher *dispatcher;
                btBroadphaseInterface *cache;
                btConstraintSolver *solver;
                btCollisionConfiguration *config;

                void setup() {
                    config = new btDefaultCollisionConfiguration();
                    dispatcher = new btCollisionDispatcher(config);
                    cache = new btDbvtBroadphase();
                    solver = new btSequentialImpulseConstraintSolver();
                    base = new btDiscreteDynamicsWorld(dispatcher, cache,
                                                       solver, config);
                    base->setGravity(btVector3(0.0f, -gravity, 0.0f));
                }

                void reset() {
                    delete base;
                    delete solver;
                    delete cache;
                    delete dispatcher;
                    delete config;
                }
            } world;
        public:
            BodyImp() {
                mMat = mat4(1.0f);
                world.setup();
            }

            ~BodyImp() {
                reset();
                world.reset();
            }

            void loadModel(const pmx::Model *m) {
                model = m;
                int n = m->rigids.size();
                rigids.resize(n);
                for (int i = 0; i < n; ++i) {
                    rigids[i].load(m, i);
                    int group = 1 << ((int)rigids[i].base->group);
                    int mask = (int)rigids[i].base->mask;
                    world.base->addRigidBody(rigids[i].bt, group, mask);
                }
                n = m->joints.size();
                joints.resize(n);
                for (int i = 0; i < n; ++i) {
                    const auto &joint = m->joints[i];
                    mat4 W = mmd2gl(joint.position, joint.rotation);
                    mat4 a = inverse(rigids[joint.rigidA].W0) * W;
                    btGeneric6DofSpring2Constraint *j;
                    if (joint.rigidB >= 0) {
                        mat4 b = inverse(rigids[joint.rigidB].W0) * W;
                        j = new btGeneric6DofSpring2Constraint(
                            *(rigids[joint.rigidA].bt),
                            *(rigids[joint.rigidB].bt),
                            gl2bt(a), gl2bt(b)
                        );
                    } else {
                        j = new btGeneric6DofSpring2Constraint(
                            *(rigids[joint.rigidA].bt), gl2bt(a)
                        );
                    }
                    j->setLinearLowerLimit(gl2bt(joint.posMin));
                    j->setLinearUpperLimit(gl2bt(joint.posMax));
                    j->setAngularLowerLimit(gl2bt(joint.rotMin));
                    j->setAngularUpperLimit(gl2bt(joint.rotMax));
                    const vec3 &ps = joint.posSpring;
                    const vec3 &rs = joint.rotSpring;
                    float s[] = {
                        ps[0], ps[1], ps[2],
                        rs[0], rs[1], rs[2]
                    };
                    for (int k = 0; k < 6; ++k) {
                        if (s[k] > 0.0f) {
                            j->enableSpring(k, true);
                            j->setStiffness(k, s[k]);
                        }
                    }
                    world.base->addConstraint(j);
                    joints[i] = j;
                }
            }

            void bindArmature(Armature *armature) {
                this->armature = armature;
            }

            void reset() {
                int n = rigids.size();
                for (int i = 0; i < n; ++i) {
                    world.base->removeRigidBody(rigids[i].bt);
                    rigids[i].reset();
                }
                rigids.clear();
                n = joints.size();
                for (int i = 0; i < n; ++i) {
                    world.base->removeConstraint(joints[i]);
                    delete joints[i];
                }
                joints.clear();
            }

            void resetPose() {
                int n = rigids.size();
                for (int i = 0; i < n; ++i) {
                    int bone = rigids[i].base->bone;
                    if (bone >= 0 && rigids[i].base->mode) {
                        armature->applyLocal(bone, mat4(1.0f));
                        mat4 w = mMat * armature->skin(bone) * rigids[i].W0;
                        rigids[i].motion->setWorldTransform(gl2bt(w));
                    }
                }
            }

            void applyBone() {
                int n = rigids.size();
                for (int i = 0; i < n; ++i) {
                    int bone = rigids[i].base->bone;
                    if (bone >= 0 && !rigids[i].base->mode) {
                        mat4 w = mMat * armature->skin(bone) * rigids[i].W0;
                        rigids[i].motion->setWorldTransform(gl2bt(w));
                    }
                }
            }

            void applyGlobal(const mat4 &m) {
                mMat = m;
            }

            void stepSimulation(float tick) {
                world.base->stepSimulation(tick, maxSubstep, frequency);
            }

            void updateBone() {
                int n = rigids.size();
                for (int i = 0; i < n; ++i) {
                    int bone = rigids[i].base->bone;
                    if (bone >= 0 && rigids[i].base->mode) {
                        btTransform Wt;
                        rigids[i].motion->getWorldTransform(Wt);
                        mat4 W = bt2gl(Wt);
                        mat4 Pi(1.0f);
                        int p = model->bones[bone].parent;
                        if (p >= 0) Pi = inverse(armature->global(p));
                        mat4 L = rigids[i].Ti * Pi * inverse(mMat) * W * rigids[i].S0i;
                        armature->applyLocal(bone, L);
                    }
                }
            }

            void update(float tick) {
                applyBone();
                btTransform t;
                rigids[23].motion->getWorldTransform(t);
                LOG << bt2gl(t.getOrigin());
                stepSimulation(tick);
                rigids[23].motion->getWorldTransform(t);
                LOG << bt2gl(t.getOrigin());
                updateBone();
            }
        };

        Body *Body::create() {
            return new BodyImp();
        }

    } /* physics */
} /* mmd */
