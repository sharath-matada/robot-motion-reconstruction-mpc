#ifndef ROBOTOC_UNCONSTR_STATE_EQUATION_HPP_
#define ROBOTOC_UNCONSTR_STATE_EQUATION_HPP_

#include "Eigen/Core"

#include "robotoc/core/split_solution.hpp"
#include "robotoc/core/split_kkt_residual.hpp"
#include "robotoc/core/split_kkt_matrix.hpp"


namespace robotoc {

///
/// @brief Linearizes the state equation of forward Euler. 
/// @param[in] dt Time step. 
/// @param[in] s Solution at the current stage. 
/// @param[in] s_next Solution at the next time stage. 
/// @param[in, out] kkt_matrix Split KKT matrix at the current time stage. 
/// @param[in, out] kkt_residual Split KKT residual at the current time stage. 
///
void linearizeUnconstrForwardEuler(const double dt, const SplitSolution& s, 
                                   const SplitSolution& s_next, 
                                   SplitKKTMatrix& kkt_matrix, 
                                   SplitKKTResidual& kkt_residual);

///
/// @brief Linearizes the state equation of forward Euler. 
/// @param[in] s Solution at the current stage. 
/// @param[in, out] kkt_residual Split KKT residual at the current time stage. 
///
void linearizeUnconstrForwardEulerTerminal(const SplitSolution& s, 
                                           SplitKKTResidual& kkt_residual);

///
/// @brief Linearizes the state equation of backward Euler. 
/// @param[in] dt Time step. 
/// @param[in] q_prev Configuration at the previous time stage. 
/// @param[in] v_prev Generalized velocity at the previous time stage. 
/// @param[in] s Solution at the current tiem stage. 
/// @param[in] s_next Solution at the next time stage. 
/// @param[in, out] kkt_matrix Split KKT matrix at the current time stage. 
/// @param[in, out] kkt_residual Split KKT reisdual at the current time stage. 
///
void linearizeUnconstrBackwardEuler(const double dt, 
                                    const Eigen::VectorXd& q_prev, 
                                    const Eigen::VectorXd& v_prev,
                                    const SplitSolution& s, 
                                    const SplitSolution& s_next, 
                                    SplitKKTMatrix& kkt_matrix, 
                                    SplitKKTResidual& kkt_residual);

///
/// @brief Linearizes the state equation of backward Euler at the terminal stage. 
/// @param[in] dt Time step. 
/// @param[in] q_prev Configuration at the previous time stage. 
/// @param[in] v_prev Generalized velocity at the previous time stage. 
/// @param[in] s Solution at the current tiem stage. 
/// @param[in, out] kkt_matrix Split KKT matrix at the current time stage. 
/// @param[in, out] kkt_residual Split KKT reisdual at the current time stage. 
///
void linearizeUnconstrBackwardEulerTerminal(const double dt, 
                                            const Eigen::VectorXd& q_prev, 
                                            const Eigen::VectorXd& v_prev, 
                                            const SplitSolution& s, 
                                            SplitKKTMatrix& kkt_matrix, 
                                            SplitKKTResidual& kkt_residual);

///
/// @brief Computes the residual in the state equation of forward Euler. 
/// @param[in] dt Time step. 
/// @param[in] s Solution at the current time stage. 
/// @param[in] s_next Solution at the next time stage. 
/// @param[in, out] kkt_residual Split KKT residual at the current time stage. 
///
void evalUnconstrForwardEuler(const double dt, const SplitSolution& s, 
                              const SplitSolution& s_next, 
                              SplitKKTResidual& kkt_residual);

///
/// @brief Computes the residual in the state equation of backward Euler. 
/// @param[in] dt Time step. 
/// @param[in] q_prev Configuration at the previous time stage. 
/// @param[in] v_prev Generalized velocity at the previous time stage. 
/// @param[in] s Solution at the current time stage. 
/// @param[in, out] kkt_residual Split KKT residual at the current time stage. 
///
void evalUnconstrBackwardEuler(const double dt, const Eigen::VectorXd& q_prev, 
                               const Eigen::VectorXd& v_prev, 
                               const SplitSolution& s, 
                               SplitKKTResidual& kkt_residual);

} // namespace robotoc 

#endif // ROBOTOC_UNCONSTR_STATE_EQUATION_HPP_ 