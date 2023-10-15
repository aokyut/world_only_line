#pragma once
#include <iostream>
#include "LineBody.h"
#include "Constraint.h"
#include "Bar.h"
#include <Eigen/Dense>
// #include <Eigen/LU>

using namespace std;

namespace physics
{

    struct FloorCollisionJacobian
    {
        int id;
        int vix;
        int viy;
        int wi;
    };

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
        float fric_mu = 0.2;
        float th_gauss_seidel = 0.00001;
        float loops_gauss_seidel = 100;
        float floor_y = -2.0;
        Bar floor;

    public:
        World() : body_count(0), floor(Bar())
        {
            wsize = 0;
            delta = 0.002;
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

            vector<FloorCollisionJacobian> collision_jacobians;

            // 衝突を数える
            for (LineBody *body : bodies)
            {
                if (body->s(1) <= floor_y)
                {
                    Vector2f r = body->s - body->c;
                    struct FloorCollisionJacobian j1 = {body->_getId(), 0, 1, r(0)};
                    struct FloorCollisionJacobian j2 = {body->_getId(), 1, 0, -r(1)};
                    collision_jacobians.push_back(j1);
                    collision_jacobians.push_back(j2);
                }
                if (body->t(1) <= floor_y)
                {
                    Vector2f r = body->t - body->c;
                    struct FloorCollisionJacobian j1 = {body->_getId(), 0, 1, r(0)};
                    struct FloorCollisionJacobian j2 = {body->_getId(), 1, 0, -r(1)};
                    collision_jacobians.push_back(j1);
                    collision_jacobians.push_back(j2);
                }
            }

            int collision_dim = collision_jacobians.size();
            cout << "collision dim: " << collision_dim << endl;

            // 拘束を処理する
            // [w_size, body * 3]
            MatrixXf jacobian = MatrixXf::Zero(wsize + collision_dim, body_count * 3);
            // MatrixXf jacobian = MatrixXf::Zero(wsize, body_count * 3);
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

            for (FloorCollisionJacobian cjacob : collision_jacobians)
            {
                int offset = cjacob.id * 3;
                jacobian(jacob_w_index, offset) = cjacob.vix;
                jacobian(jacob_w_index, offset + 1) = cjacob.viy;
                jacobian(jacob_w_index, offset + 2) = cjacob.wi;

                jacob_w_index++;
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
            // cout << "Here is the vector jacob:\n"
            //      << jacobian << endl;

            // 拘束力を求める
            MatrixXf A = jacobian * m_inv_mat * jacobian.transpose() * delta;
            VectorXf b = jacobian * (u + m_inv_mat * f_ex * delta);
            // -b = Aλ

            // VectorXf lambda = A.colPivHouseholderQr().solve(-b);
            VectorXf lambda = solve_gauss_seidel(A, b, collision_dim, wsize);
            // VectorXf _lambda = A.colPivHouseholderQr().solve(-b);

            // cout << "Solve Error is"
            //      << (lambda - _lambda).squaredNorm() << endl;

            VectorXf next_u = u + m_inv_mat * (jacobian.transpose() * lambda + f_ex) * delta;

            // cout << "Here is the matrix A:\n"
            //      << A << endl;
            // cout << "Here is the vector b:\n"
            //      << b << endl;
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
            // float energy = 0;
            for (int i = 0; i < body_count; i++)
            {
                int i3 = i * 3;
                LineBody *body = bodies[i];
                body->setVelocity(next_u(i3), next_u(i3 + 1), next_u(i3 + 2));
                // if (i == 0)
                // {
                //     cout << body->_getId() << "pos-s: " << body->s << ", pos-t: " << body->t << endl;
                // }

                body->update(delta);
                // energy += body->potential();
            }
            // cout << "potential" << energy << endl;
        }

        /// @brief solve w = Aλ + b,
        /// A[0, dim_hinge]: hinge
        /// A[dim_hinge, dim_hinge + dim_col]: collisions
        /// @param A
        /// @param b
        /// @param dim_col
        /// @param dim_hinge
        /// @param loops
        /// @return
        VectorXf solve_gauss_seidel(MatrixXf A, VectorXf b, int dim_col, int dim_hinge)
        {
            int dim = dim_col + dim_hinge;
            // A, bを整形
            for (int i = 0; i < dim; i++)
            {
                float Aii = A(i, i);
                A(i, i) = 0;
                for (int j = 0; j < dim; j++)
                {
                    A(i, j) = -A(i, j) / Aii;
                }
                b(i) = -b(i) / Aii;
            }

            // if (dim_col != 0)
            // {
            //     cout << "Here is the matrix A:\n"
            //          << A << endl;
            //     cout << "Here is the vector b:\n"
            //          << b << endl;
            // }

            VectorXf lambda = VectorXf::Zero(dim);
            float d_lambda;
            int i;
            for (i = 0; i < loops_gauss_seidel; i++)
            {
                d_lambda = 0;
                // process hinge_joints
                int j = 0;
                for (; j < dim_hinge; j += 2)
                {
                    const float lambda_j = (A.row(j).dot(lambda) + b(j));
                    // d_lambda += abs(lambda_j - lambda(j));
                    lambda(j) = lambda_j;
                    const float lambda_j1 = (A.row(j + 1).dot(lambda) + b(j + 1));
                    // cout << "b(j+1):" << b(j + 1) << endl;
                    d_lambda += abs(lambda_j1 + lambda_j - lambda(j + 1) - lambda(j));
                    lambda(j + 1) = lambda_j1;
                }

                // cout << "Here is the vector lambda:\n"
                //      << lambda << endl;
                // process collisions
                for (; j < dim; j += 2)
                {
                    const float lambda_j = max(0.0f, (A.row(j).dot(lambda) + b(j)));
                    // d_lambda += abs(lambda_j - lambda(j));
                    lambda(j) = lambda_j;
                    const float lambda_j1 = max(-fric_mu * lambda(j), min(fric_mu * lambda(j), (A.row(j + 1).dot(lambda) + b(j + 1))));
                    d_lambda += abs(lambda_j1 + lambda_j - lambda(j + 1) - lambda(j));
                }
                if (d_lambda / dim < th_gauss_seidel)
                {
                    break;
                }
            }
            cout << "loops:" << i << "  d_lambda:" << d_lambda / dim << endl;
            return lambda;
        }

        void render()
        {
            for (int i = 0; i < body_count; i++)
            {
                bodies[i]->draw();
            }
            Vector2f s(-10, floor_y);
            // s << -10, floor_y;
            Vector2f t(10, floor_y);
            // t << 10, floor_y;
            floor.draw(s, t);
        }
    };
}