#include "ieskf.h"

double State::gravity = 9.81;

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const V21D &delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
}

V21D State::operator-(const State &other) const
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_il: " << state.t_il.transpose() << std::endl;
    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    os << "v: " << state.v.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt;
    delta.segment<3>(3) = m_x.v * dt;
    delta.segment<3>(12) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro - m_x.bg) * dt).matrix();
    m_F.block<3, 3>(0, 15) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_F.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;
    m_F.block<3, 3>(12, 18) = -m_x.r_wi * dt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_G.block<3, 3>(12, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    m_x += delta;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}

void IESKF::update()
{
    State predict_x = m_x;
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    V21D last_delta = V21D::Zero();
    M21D posterior_information = M21D::Zero();
    bool update_applied = false;

    Eigen::LDLT<M21D> prior_solver(m_P);
    if (prior_solver.info() != Eigen::Success || !prior_solver.isPositive())
        return;
    const M21D prior_information = prior_solver.solve(M21D::Identity());
    if (prior_solver.info() != Eigen::Success || !prior_information.allFinite())
        return;

    for (size_t i = 0; i < m_max_iter; i++)
    {
        shared_data.valid = false;
        m_loss_func(m_x, shared_data);
        if (!shared_data.valid)
            break;
        const V21D state_delta = m_x - predict_x;
        M21D J = M21D::Identity();
        J.block<3, 3>(0, 0) = JrInv(state_delta.segment<3>(0));
        J.block<3, 3>(6, 6) = JrInv(state_delta.segment<3>(6));
        M21D information = J.transpose() * prior_information * J;
        V21D b = J.transpose() * prior_information * state_delta;

        information.block<12, 12>(0, 0) += shared_data.H;
        b.block<12, 1>(0, 0) += shared_data.b;

        Eigen::LDLT<M21D> information_solver(information);
        if (information_solver.info() != Eigen::Success || !information_solver.isPositive())
            break;
        const V21D delta = -information_solver.solve(b);
        if (information_solver.info() != Eigen::Success || !delta.allFinite())
            break;

        m_x += delta;
        posterior_information = information;
        last_delta = delta;
        update_applied = true;
        shared_data.iter_num += 1;

        if (m_stop_func(delta))
            break;
    }

    if (!update_applied)
        return;

    M21D L = M21D::Identity();
    // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
    // L.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
    L.block<3, 3>(0, 0) = Jr(last_delta.segment<3>(0));
    L.block<3, 3>(6, 6) = Jr(last_delta.segment<3>(6));
    Eigen::LDLT<M21D> posterior_solver(posterior_information);
    if (posterior_solver.info() != Eigen::Success || !posterior_solver.isPositive())
    {
        m_x = predict_x;
        return;
    }
    const M21D covariance = L * posterior_solver.solve(M21D::Identity()) * L.transpose();
    if (posterior_solver.info() == Eigen::Success && covariance.allFinite())
        m_P = 0.5 * (covariance + covariance.transpose());
    else
        m_x = predict_x;
}
