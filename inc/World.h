#pragma once
#include <iostream>
#include "LineBody.h"
#include "Constraint.h"
#include <Eigen/Dense>
// #include <Eigen/LU>

using namespace std;

namespace physics
{
    class World
    {
    private:
        vector<LineBody *> bodies;
        vector<Constraint *> constraints;
        int body_count;
        int wsize;
        int id_max;
        float delta;
        float penalty_alpha;
        float penalty_beta;

    public:
        World() : body_count(0)
        {
            wsize = 0;
            delta = 0.01;
            id_max = 0;
            penalty_alpha = 0.1;
            penalty_beta = 0.02;
        }

        void addBody(LineBody *body)
        {
            body->_setId(id_max);
            id_max++;
            cout << "id:" << body->_getId() << endl;
            bodies.push_back(body);
            body_count++;
        }
        void addJoint(Constraint *constraint)
        {
            constraints.push_back(constraint);
            wsize += constraint->wsize();
        }
        void step()
        {
            // int cmax = constraints.size();

            // 拘束を処理する
            // [w_size, body * 3]
            MatrixXf jacobian = MatrixXf::Zero(wsize, body_count * 3);
            MatrixXf m_mat = MatrixXf::Zero(body_count * 3, body_count * 3);
            MatrixXf m_inv_mat = MatrixXf::Zero(body_count * 3, body_count * 3);
            // 　外力
            VectorXf f_ex = VectorXf::Zero(body_count * 3);
            // 絶対速度
            VectorXf u = VectorXf::Zero(body_count * 3);

            int jacob_w_index = 0;
            // set jacobian
            for (Constraint *c : constraints)
            {
                c->fix(penalty_alpha, penalty_beta);
                for (CJacobian cjacob : c->getJacobian())
                {
                    int offset_i = cjacob.iid * 3;
                    int offset_j = cjacob.jid * 3;
                    jacobian(jacob_w_index, offset_i) = cjacob.vix;
                    jacobian(jacob_w_index, offset_i + 1) = cjacob.viy;
                    jacobian(jacob_w_index, offset_i + 2) = cjacob.wi;

                    jacobian(jacob_w_index, offset_j) = cjacob.vjx;
                    jacobian(jacob_w_index, offset_j + 1) = cjacob.vjy;
                    jacobian(jacob_w_index, offset_j + 2) = cjacob.wj;

                    jacob_w_index++;
                }
            }

            // set m
            for (int i = 0; i < 3 * body_count; i += 3)
            {
                LineBody *body = bodies[i / 3];
                float I = body->I;
                float m = body->m;
                m_mat(i, i) = m;
                m_mat(i + 1, i + 1) = m;
                m_mat(i + 2, i + 2) = I;
                m_inv_mat(i, i) = 1 / m;
                m_inv_mat(i + 1, i + 1) = 1 / m;
                m_inv_mat(i + 2, i + 2) = 1 / I;
                f_ex(i) = body->f_ex(0);
                f_ex(i + 1) = body->f_ex(1);
                f_ex(i + 2) = body->fw;
                u(i) = body->v(0);
                u(i + 1) = body->v(1);
                u(i + 2) = body->w;
            }

            // 拘束力を求める
            MatrixXf A = jacobian * m_inv_mat * jacobian.transpose() * delta;
            VectorXf b = jacobian * (u + m_inv_mat * f_ex * delta);
            // -b = Aλ
            VectorXf lambda = A.colPivHouseholderQr().solve(-b);

            VectorXf next_u = u + m_inv_mat * (jacobian.transpose() * lambda + f_ex) * delta;

            // cout << "Here is the matrix A:\n"
            //      << A << endl;
            // cout << "Here is the vector b:\n"
            //      << b << endl;
            // cout << "Here is the vector jacob:\n"
            //      << jacobian << endl;
            // cout << "Here is the vector lambda:\n"
            //      << lambda << endl;

            // cout << "M_inv * J_t:\n"
            //      << m_inv_mat * jacobian.transpose() << endl;
            // cout << " u + m-1 * f * delta:\n"
            //      << u + m_inv_mat * f_ex * delta << endl;

            // cout << "Here is the vector constraint force:\n"
            //      << jacobian.transpose() * lambda << endl;

            // cout << "The force is:\n"
            //      << f_ex << endl;
            float energy = 0;
            for (int i = 0; i < body_count; i++)
            {
                int i3 = i * 3;
                LineBody *body = bodies[i];
                body->setVelocity(next_u(i3), next_u(i3 + 1), next_u(i3 + 2));
                // cout << body->_getId() << "pos-s: " << body->s << ", pos-t: " << body->t << endl;

                body->update(delta);
                energy += body->potential();
            }
            cout << "potential" << energy << endl;
        }
        void render()
        {
            for (int i = 0; i < body_count; i++)
            {
                bodies[i]->draw();
            }
        }
    };
}