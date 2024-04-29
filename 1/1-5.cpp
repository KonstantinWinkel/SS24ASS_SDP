#include <nlohmann/json.hpp>
#include <Eigen/Core>
#include <iostream>
#include <fstream>

using json = nlohmann::json;


void readIMU(const std::string &infile, Eigen::ArrayXd &t, Eigen::ArrayXd &ax, Eigen::ArrayXd &ay,
    Eigen::ArrayXd &az, Eigen::ArrayXd &wx, Eigen::ArrayXd &wy, Eigen::ArrayXd &wz)
{
    std::ifstream ifs(infile, std::ifstream::in);

    std::vector<double> _t, _ax, _ay, _az, _wx, _wy, _wz;

    std::string line;
    while (std::getline(ifs, line)) {
        json data = json::parse(line);

        if (!data.contains("msg")) {
            continue;
        }

        if (data["msg"].get<std::string>().compare("imu_raw") == 0) {
            double stamp = data["stamp"].get<double>();
            unsigned long seq = data["seq"].get<unsigned long>();
            _t.push_back(data["stamp"].get<double>());
            _ax.push_back(data["ax"].get<double>());
            _ay.push_back(data["ay"].get<double>());
            _az.push_back(data["az"].get<double>());
            _wx.push_back(data["wx"].get<double>());
            _wy.push_back(data["wy"].get<double>());
            _wz.push_back(data["wz"].get<double>());
        }
    }

    t = Eigen::Map<Eigen::ArrayXd>(_t.data(), _t.size());
    ax = Eigen::Map<Eigen::ArrayXd>(_ax.data(), _ax.size());
    ay = Eigen::Map<Eigen::ArrayXd>(_ay.data(), _ay.size());
    az = Eigen::Map<Eigen::ArrayXd>(_az.data(), _az.size());
    wx = Eigen::Map<Eigen::ArrayXd>(_wx.data(), _wx.size());
    wy = Eigen::Map<Eigen::ArrayXd>(_wy.data(), _wy.size());
    wz = Eigen::Map<Eigen::ArrayXd>(_wz.data(), _wz.size());
}

inline double stdDev(Eigen::ArrayXd v){
    return std::sqrt((v - v.mean()).square().sum()/(v.size()-1));
}

void compute(std::string path){
    Eigen::ArrayXd t, ax, ay, az, wx, wy, wz;
    readIMU(path, t, ax, ay, az, wx, wy, wz);

    std::cout << "Loaded " << t.size() << " IMU messages." << std::endl;

    // process IMU data

    // compute norm of the acceleration vector
    //a = np.sqrt(ax*ax + ay*ay + az*az)
    Eigen::VectorXd a = (ax.array() * ax.array() + ay.array() * ay.array() + az.array() * az.array()).cwiseSqrt();

    double avg = a.mean();
    std::cout << avg << std::endl;

    std::cout<<"Acc X: Mean: "<< ax.mean() <<"\t" <<stdDev(ax)<<std::endl;
    std::cout<<"Acc Y: Mean: "<< ay.mean() <<"\t" <<stdDev(ay)<<std::endl;
    std::cout<<"Acc Z: Mean: "<< az.mean() <<"\t" <<stdDev(az)<<std::endl;

    std::cout<<"Gyr X: Mean: "<< wx.mean() <<"\t" <<stdDev(wx)<<std::endl;
    std::cout<<"Gyr Y: Mean: "<< wy.mean() <<"\t" <<stdDev(wy)<<std::endl;
    std::cout<<"Gyr Z: Mean: "<< wz.mean() <<"\t" <<stdDev(wz)<<std::endl;
}

int main(int argc, char *argv[])
{
    std::cout<<"kvh1750"<<std::endl;
    //Best gyro, overall best
    compute("../../data/imu_raw_kvh1750_static.json");
    std::cout<<"\n\nphidgets"<<std::endl;
    //Best accelerometer, needs offset
    compute("../../data/imu_raw_phidgets_static.json");
    std::cout<<"\n\nsensorcube"<<std::endl;
    //straight trash
    compute("../../data/imu_raw_sensorcube_static.json");
    return 0;
}
