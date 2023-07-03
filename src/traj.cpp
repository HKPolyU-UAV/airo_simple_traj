#include "include/opt_traj_gen.hpp"

static double _Vel = 0.5;
static double _Acc = 0.5;
static int _dev_order = 3;
static int _poly_num1D = 2 * _dev_order;
static std::vector<Eigen::Vector3d> trajectory;


Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t )
{
    Eigen:: Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        Eigen::VectorXd time  = Eigen::VectorXd::Zero( _poly_num1D );

        for(int j = 0; j < _poly_num1D; j ++)
            if(j==0)
                time(j) = 1.0;
            else
                time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    // cout << "ret = " << endl;
    // cout << ret << endl;

    return ret;
}

void MJTG_execute( Eigen::MatrixXd polyCoeff, Eigen::VectorXd time)//for final movement
{
    Eigen::Vector3d pos, cur, pre;
    Eigen::Vector3d traj_pt;
    int count = 0;
    double traj_len = 0.0;

    for(int i = 0; i < time.size(); i++ )   // go through each segment
    {
        for (double t = 0.0; t < time(i); t += 0.05, count += 1)
        {
            pos = getPosPoly(polyCoeff, i, t);
            cur(0) = traj_pt.x() = pos(0);
            cur(1) = traj_pt.y() = pos(1);
            cur(2) = traj_pt.z() = pos(2);
            trajectory.push_back(traj_pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }
}


Eigen::VectorXd timeAllocation( Eigen::MatrixXd Path)
{
    Eigen::VectorXd time(Path.rows() - 1);
    Eigen::VectorXd distance = Eigen::VectorXd::Zero(Path.rows()-1);
    for(int i = 0; i<Path.rows()-1;i++)
    {
        distance(i) = sqrt
            (
                pow(  (Path(i+1,0)-Path(i,0)) ,2)
                +pow(  (Path(i+1,1)-Path(i,1)) ,2)
                +pow(  (Path(i+1,2)-Path(i,2)) ,2)
                );
        double t1 = _Vel / _Acc;
        double t2 = distance(i)/_Vel - t1;
        time(i) = 2*t1 +t2 ;
    }
    return time;
}

void trajGeneration(Eigen::MatrixXd path)
{
    Eigen::MatrixXd polyCoeff;
    ros::Time time_start = ros::Time::now();

    traj::opt_traj_gen traj_gen_obj;

    Eigen::MatrixXd vel  = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd acc  = Eigen::MatrixXd::Zero(2, 3);
    vel.row(0)  = Eigen::Vector3d::Zero();;
    vel.row(1)  = Eigen::Vector3d::Zero();;
    acc.row(0)  = Eigen::Vector3d::Zero();;
    acc.row(1)  = Eigen::Vector3d::Zero();;

    // use "trapezoidal velocity" time allocation
    Eigen::VectorXd _polyTime = timeAllocation(path);


    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    int _dev_order = 3;
    Eigen::MatrixXd _polyCoeff = traj_gen_obj.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    std::cout << "_polyCoeff = " << std::endl;
    std::cout << _polyCoeff.rows() << std::endl;
    std::cout << _polyCoeff.cols() << std::endl;

    MJTG_execute(_polyCoeff, _polyTime);

    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);
    ROS_WARN("Time consumed use minimum jerk trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_test");
    ros::NodeHandle nh;

    Eigen::MatrixXd waypoints(4,3);
    waypoints.row(0) = Eigen::Vector3d(0,0,2);
    waypoints.row(1) = Eigen::Vector3d(3,0,2);
    waypoints.row(2) = Eigen::Vector3d(5,6,2);
    waypoints.row(3) = Eigen::Vector3d(0,0,2);
    trajGeneration(waypoints);

    return 0;
}