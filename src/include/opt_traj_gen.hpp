#include "essential.h"

namespace traj{
    class opt_traj_gen
    {
    private:
        double _qp_cost;
        Eigen::MatrixXd _Q, _M, _Ct;
        Eigen::VectorXd _Px, _Py, _Pz;

        Eigen::MatrixXd getQ(const int p_num1d, const int order, const Eigen::VectorXd &Time, const int seg_index);

        Eigen::MatrixXd getM(const int p_num1d, const int order, const Eigen::VectorXd &Time, const int seg_index);

        Eigen::MatrixXd getCt(const int seg_num, const int d_order);

        Eigen::VectorXd closedFormCalCoeff1D(
            const Eigen::MatrixXd &Q,
            const Eigen::MatrixXd &M,
            const Eigen::MatrixXd &Ct,
            const Eigen::VectorXd &WayPoints1D,
            const Eigen::VectorXd &StartState1D,
            const Eigen::VectorXd &EndState1D,
            const int seg_num,
            const int d_order
        );

    public:
        opt_traj_gen(/* args */);
        ~opt_traj_gen();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time
        );

        int Factorial(int x);
        double desired_yaw;
        
    };
    
    opt_traj_gen::opt_traj_gen(/* args */)
    {
        std::cout<<"OPT_TRAJ_GEN CONSTRUCTOR"<<std::endl;
    }
    
    opt_traj_gen::~opt_traj_gen()
    {
        std::cout<<"OPT_TRAJ_GEN DESTRUCTOR"<<std::endl;
    }

    Eigen::MatrixXd opt_traj_gen::getQ(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
    {
        Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
        Eigen::VectorXd time = Eigen::VectorXd::Zero(p_num1d);
        for(int i = 0; i < p_num1d; i++)
        {
            time(i) = pow(Time(seg_index),i);
        }
        if(p_num1d == 6)        // minimum jerk
        {
            Q_k << 0,     0     ,     0     ,      0     ,       0     ,       0     ,
                0,     0     ,     0     ,      0     ,       0     ,       0     ,
                0,     0     ,     0     ,      0     ,       0     ,       0     ,
                0,     0     ,     0     ,  36*time(1),   72*time(2),  120*time(3),
                0,     0     ,     0     ,  72*time(2),  192*time(3),  360*time(4),
                0,     0     ,     0     , 120*time(3),  360*time(4),  720*time(5);
        }
        else if(p_num1d == 8)   // minimum snap
        {
            Q_k << 0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
                0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
                0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
                0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
                0,     0    ,      0     ,      0     ,  576*time(1),  1440*time(2),  2880*time(3),   5040*time(4),
                0,     0    ,      0     ,      0     , 1440*time(2),  4800*time(3), 10800*time(4),  20160*time(5),
                0,     0    ,      0     ,      0     , 2880*time(3), 10800*time(4), 25920*time(5),  50400*time(6),
                0,     0    ,      0     ,      0     , 5040*time(4), 20160*time(5), 50400*time(6), 100800*time(7);
        }
        // cout << " Q_k = " << endl;
        // cout <<  Q_k << endl;

        return Q_k;
    }

    Eigen::MatrixXd opt_traj_gen::getM(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
    {
        Eigen::MatrixXd M_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
        Eigen::VectorXd time = Eigen::VectorXd::Zero(p_num1d);
        for(int i = 0; i < p_num1d; i++)
        {
            time(i) = pow(Time(seg_index),i);
        }
        if(p_num1d == 6)        // minimum jerk
        {
            M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,
                0,     1   ,     0     ,     0     ,      0     ,      0     ,
                0,     0   ,     2     ,     0     ,      0     ,      0     ,
                1,  time(1),    time(2),    time(3),     time(4),     time(5),
                0,     1   ,  2*time(1),  3*time(2),   4*time(3),   5*time(4),
                0,     0   ,     2     ,  6*time(1),  12*time(2),  20*time(3);
        }
        else if(p_num1d == 8)   // minimum snap
        {
            M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
                0,     1   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
                0,     0   ,     2     ,     0     ,      0     ,      0     ,      0     ,      0     ,
                0,     0   ,     0     ,     6     ,      0     ,      0     ,      0     ,      0     ,
                1,  time(1),    time(2),    time(3),     time(4),     time(5),     time(6),     time(7),
                0,     1   ,  2*time(1),  3*time(2),   4*time(3),   5*time(4),   6*time(5),   7*time(6),
                0,     0   ,     2     ,  6*time(1),  12*time(2),  20*time(3),  30*time(4),  42*time(5),
                0,     0   ,     0     ,     6     ,  24*time(1),  60*time(2), 120*time(3), 210*time(4);
        }
        // cout << "M_k = " << endl;
        // cout << M_k << endl;

        return M_k;
    }

    Eigen::MatrixXd opt_traj_gen::getCt(const int n_seg, const int d_order)
    {
        int d_num = 2 * d_order * n_seg;
        int dF_dP_num = (n_seg + 1) * d_order;

        Eigen::MatrixXd Ct = Eigen::MatrixXd::Zero(d_num, dF_dP_num);

        Eigen::MatrixXd C_temp = Eigen::MatrixXd::Zero(2 * d_order, dF_dP_num);

        Ct.block(0, 0, d_order, d_order) = Eigen::MatrixXd::Identity(d_order, d_order);

        if(d_order == 3)//use minimum jerk
        {
            for(int k = 0; k < n_seg - 1; k++)
            {
                C_temp(0, d_order + k) = 1;
                C_temp(1, d_order + d_order + (n_seg - 1) + 2*k) = 1;
                C_temp(2, d_order + d_order + (n_seg - 1) + 2*k + 1) = 1;

                C_temp(3, d_order + k) = 1;
                C_temp(4, d_order + d_order + (n_seg - 1) + 2*k) = 1;
                C_temp(5, d_order + d_order + (n_seg - 1) + 2*k + 1) = 1;

                Ct.block(d_order + 2 * d_order * k, 0, 2 * d_order, dF_dP_num) = C_temp;
                C_temp = Eigen::MatrixXd::Zero(2 * d_order, dF_dP_num);
            }
        }

        if(d_order == 4)//use minimum snap
        {
            for(int k = 0; k < n_seg - 1; k++)
            {
                C_temp(0, d_order + k) = 1;
                C_temp(1, d_order + d_order + (n_seg - 1) + 3*k) = 1;
                C_temp(2, d_order + d_order + (n_seg - 1) + 3*k + 1) = 1;
                C_temp(3, d_order + d_order + (n_seg - 1) + 3*k + 2) = 1;

                C_temp(4, d_order + k) = 1;
                C_temp(5, d_order + d_order + (n_seg - 1) + 3*k) = 1;
                C_temp(6, d_order + d_order + (n_seg - 1) + 3*k + 1) = 1;
                C_temp(7, d_order + d_order + (n_seg - 1) + 3*k + 2) = 1;

                Ct.block(d_order + 2 * d_order * k, 0, 2 * d_order, dF_dP_num) = C_temp;
                C_temp = Eigen::MatrixXd::Zero(2 * d_order, dF_dP_num);
            }
        }
        Ct.block((n_seg - 1) * 2 * d_order + d_order , d_order + (n_seg - 1) , d_order, d_order) = Eigen::MatrixXd::Identity(d_order, d_order);

        // cout << "Ct = " << endl;
        // cout << Ct << endl;
        return Ct;
    }

    Eigen::VectorXd opt_traj_gen::closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                                                    const Eigen::MatrixXd &M,
                                                                    const Eigen::MatrixXd &Ct,
                                                                    const Eigen::VectorXd &WayPoints1D,
                                                                    const Eigen::VectorXd &startPointState1D,
                                                                    const Eigen::VectorXd &endPointState1D,
                                                                    const int n_seg,
                                                                    const int d_order)
    {
        int dF_dP_num = d_order * (n_seg + 1);

        int df_num = 2 * d_order + (n_seg - 1);
        int dp_num = (d_order - 1) * (n_seg - 1);

        Eigen::MatrixXd C = Ct.transpose();
        Eigen::MatrixXd M_inv = M.inverse();
        Eigen::MatrixXd M_inv_tran = M_inv.transpose();

        Eigen::MatrixXd R = C * M_inv_tran * Q * M_inv * Ct;
        Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
        Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);


        Eigen::VectorXd dF(df_num);
        dF.head(d_order) = startPointState1D;//start state:p0,v0,a0,j0
        dF.segment(d_order, (n_seg - 1)) = WayPoints1D.segment(1,WayPoints1D.rows()-2);
        dF.tail(d_order) = endPointState1D;//end state:pf,vf,af,jf
        

        Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;//closed form



        Eigen::VectorXd dF_and_dP(dF_dP_num);
        dF_and_dP << dF, dP;

        Eigen::VectorXd PolyCoeff1D = M_inv * Ct * dF_and_dP;//on same direction

        return PolyCoeff1D;
    }

    int opt_traj_gen::Factorial(int x)
    {
        int fac = 1;
        for(int i = x; i > 0; i--)
            fac = fac * i;
        return fac;
    }

    Eigen::MatrixXd opt_traj_gen::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time
    )          
    // time allocation in each segment
    {
        // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
        int p_order   = 2 * d_order - 1;              // the order of polynomial
        int p_num1d   = p_order + 1;                  // the number of variables in each segment
        int n_seg = Time.size();
        
        // the number of segments
        Eigen::MatrixXd PolyCoeff = Eigen::MatrixXd::Zero(n_seg, 3 * p_num1d);// position(x,y,z), so we need (3 * p_num1d) coefficients

        Eigen::VectorXd Px(p_num1d * n_seg);
        Eigen::VectorXd Py(p_num1d * n_seg);
        Eigen::VectorXd Pz(p_num1d * n_seg);

        Eigen::MatrixXd startpt(d_order,3);
        Eigen::MatrixXd endpt(d_order,3);

        startpt.row(0)=Path.row(0);
        startpt.row(1)=Vel.row(0);
        startpt.row(2)=Acc.row(0);
        endpt.row(0)=Path.row(Path.rows()-1);
        endpt.row(1)=Vel.row(1);
        endpt.row(2)=Acc.row(1);

        if(d_order == 4)//use minimum snap
        {
            startpt.row(3) = Eigen::VectorXd::Zero(3); //j0 = 0
            endpt.row(3) = Eigen::VectorXd::Zero(3);//jf = 0
        }
    //instantiate Q M & Ct


        _Q = Eigen::MatrixXd::Zero(p_num1d * n_seg, p_num1d * n_seg);
        _M = Eigen::MatrixXd::Zero(p_num1d * n_seg, p_num1d * n_seg);
        _Ct = Eigen::MatrixXd::Zero(2 * d_order * n_seg, d_order * (n_seg + 1));
        for(int seg_index = 0; seg_index < n_seg; seg_index++)
        {
            _Q.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getQ(p_num1d, d_order, Time, seg_index);
            _M.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getM(p_num1d, d_order, Time, seg_index);
        }

        _Ct = getCt(n_seg, d_order);
        Px = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(0), startpt.col(0), endpt.col(0), n_seg, d_order);
        Py = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(1), startpt.col(1), endpt.col(1), n_seg, d_order);
        Pz = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(2), startpt.col(2), endpt.col(2), n_seg, d_order);

        for(int i = 0; i < n_seg; i++)
        {
            PolyCoeff.row(i).segment(0, p_num1d) = Px.segment(p_num1d*i, p_num1d);
            PolyCoeff.row(i).segment(p_num1d, p_num1d) = Py.segment(p_num1d*i, p_num1d);
            PolyCoeff.row(i).segment(2*p_num1d, p_num1d) = Pz.segment(p_num1d*i, p_num1d); //segment refers to starting from that index, and extend for howmany
        }


        return PolyCoeff;
    }
    
}



