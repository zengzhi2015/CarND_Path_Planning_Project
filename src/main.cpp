#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "JMT.h"
#include "COST.h"
#include <mgl2/mgl.h>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2) {
    // Euclidean distance
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {
    // Checkout the nearest way point to the location [x,y]
    // and return the index of the nearest way point
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
    // If the car is not toward the nearest way point, the next way point is the next point to the nearest way point.
    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = abs(theta - heading);

    if (angle > pi() / 4) {
        closestWaypoint++;
    }

    return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
    // sd = getFrenet(...)
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
    // get the previous way point
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    // Get the next way point
    int wp2 = (prev_wp + 1) % maps_x.size();

    // Get the direction of the vector from the previous way point to the next
    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // Get the perpendicular direction
    double perp_heading = heading - pi() / 2;

    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]); // the s distance from the previous way point

    double x = maps_x[prev_wp] + seg_s * cos(heading) + d * cos(perp_heading);
    double y = maps_y[prev_wp] + seg_s * sin(heading) + d * sin(perp_heading);

    return {x, y};
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x; // modified data
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

/*    // modification test
    for(int i=0;i<map_waypoints_x.size();i++) {
        map_waypoints_x[i] += 10*map_waypoints_dx[i];
        map_waypoints_y[i] += 10*map_waypoints_dy[i];
    }*/

    // 1. Modify the way points
    // 1.1. interpolate original way points using spline tool
    tk::spline s_x, s_y, s_dx, s_dy;
    s_x.set_points(map_waypoints_s,map_waypoints_x);
    s_y.set_points(map_waypoints_s,map_waypoints_y);
    s_dx.set_points(map_waypoints_s,map_waypoints_dx);
    s_dy.set_points(map_waypoints_s,map_waypoints_dy);

    // 1.2. Rescale way points
    // 1.2.1 Modify the way points
    double s_max = *max_element(begin(map_waypoints_s),end(map_waypoints_s));
    map_waypoints_x.clear();
    map_waypoints_y.clear();
    map_waypoints_dx.clear();
    map_waypoints_dy.clear();
    for(double s=0;s<s_max;s+=1) {
        map_waypoints_x.push_back(s_x(s));
        map_waypoints_y.push_back(s_y(s));
        map_waypoints_dx.push_back(s_dx(s));
        map_waypoints_dy.push_back(s_dy(s));
    }

    // 1.2.2. Rescale way points
    map_waypoints_s.clear();
    for(int i=0;i<map_waypoints_x.size();i++) {
        if(i==0) {
            map_waypoints_s.push_back(0.0);
        }
        else {
            double delta_s = distance(map_waypoints_x[i],map_waypoints_y[i],map_waypoints_x[i-1],map_waypoints_y[i-1]);
            map_waypoints_s.push_back(map_waypoints_s[i-1]+delta_s);
        }
    }

    // 1.4. Modify the interpolation
//    tk::spline s_x_mod, s_y_mod, s_dx_mod, s_dy_mod;
//    s_x_mod.set_points(map_waypoints_s,map_waypoints_x);
//    s_y_mod.set_points(map_waypoints_s,map_waypoints_y);
//    s_dx_mod.set_points(map_waypoints_s,map_waypoints_dx);
//    s_dy_mod.set_points(map_waypoints_s,map_waypoints_dy);
    s_x.set_points(map_waypoints_s,map_waypoints_x);
    s_y.set_points(map_waypoints_s,map_waypoints_y);
    s_dx.set_points(map_waypoints_s,map_waypoints_dx);
    s_dy.set_points(map_waypoints_s,map_waypoints_dy);

    int flag=1;
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<double> next_s_vals;
    vector<double> next_d_vals;

    h.onMessage([&flag, &next_x_vals, &next_y_vals, &next_s_vals, &next_d_vals,
                        &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
                        &s_x, &s_y, &s_dx, &s_dy](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"]; // the direction of the car in degree
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"]; // 47~48 data points
                    auto previous_path_y = j[1]["previous_path_y"];

                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    // cout << sensor_fusion << endl;

                    json msgJson;

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

                    while(next_x_vals.size()>previous_path_x.size()) {
                        next_x_vals.erase(next_x_vals.begin());
                        next_y_vals.erase(next_y_vals.begin());
                        next_d_vals.erase(next_d_vals.begin());
                        next_s_vals.erase(next_s_vals.begin());
                    }


                    if(next_x_vals.size()<10) {

                        // 2.2.1. Get raw JMT trajectory
                        // Suppose the period T is 2 second.
                        // The number of path points is: 2/0.02 = 100
                        vector<double> start_s;
                        vector<double> end_s;
                        vector<double> start_d;
                        vector<double> end_d;
                        if(previous_path_x.size()==0) {
                            start_s = {car_s,0,0};
                            end_s = {car_s+50,48/2.24,0};
                            start_d = {car_d,0,0};
                            end_d = {car_d+flag*0,0,0};
                        }
                        else {
                            start_s = {next_s_vals[next_x_vals.size()-1],48/2.24,0};
                            end_s = {next_s_vals[next_x_vals.size()-1]+120,48/2.24,0};
                            start_d = {next_d_vals[next_d_vals.size()-1],0,0};
                            end_d = {next_d_vals[next_d_vals.size()-1]+flag*4,0,0};
                        }

                        flag *= -1;
                        //cout << flag << endl;
                        //cout << car_d+flag*4 << endl;
                        double T = 6;
                        JMT jmt_s, jmt_d;
                        jmt_s.cal_coefficients(start_s,end_s,T);
                        jmt_d.cal_coefficients(start_d,end_d,T);
                        COST cost;
                        cout << "collision_cost: " << cost.collision_cost(jmt_s,jmt_d,T,sensor_fusion) << endl;
                        cout << "speed_cost: " << cost.speed_cost(jmt_s,T) << endl;
                        cout << "acceleration_cost: " << cost.acceleration_cost(jmt_s,T) << endl;
                        cout << "jerk_cost: " << cost.jerk_cost(jmt_s,T) << endl;
                        cout << "efficiency_cost: " << cost.efficiency_cost(jmt_s,T) << endl;
                        cout << "total_cost: " << cost.total_cost(jmt_s,jmt_d,T,sensor_fusion) << endl;

                        vector<double> path_points_s;
                        vector<double> path_points_d;
                        vector<double> path_points_x;
                        vector<double> path_points_y;

                        for(double t=0.02;t<T;t+=0.02) {
                            double temp_s = jmt_s.F(t);
                            double temp_d = jmt_d.F(t);
                            double temp_x = s_x(temp_s) + temp_d*s_dx(temp_s);
                            double temp_y = s_y(temp_s) + temp_d*s_dy(temp_s);
                            //cout << temp_s << '\t' << endl;
                            path_points_s.push_back(temp_s);
                            path_points_d.push_back(temp_d);
                            path_points_x.push_back(temp_x);
                            path_points_y.push_back(temp_y);
                        }

                        // 2.2.2. Rescale the JMT trajectory
                        vector<double> path_points_s_rescale;
                        for(int i=0;i<path_points_s.size();i++) {
                            if(i==0) {
                                path_points_s_rescale.push_back(path_points_s[0]);
                            }
                            else {
                                double delta_s = distance(path_points_x[i],path_points_y[i],path_points_x[i-1],path_points_y[i-1]);
                                path_points_s_rescale.push_back(path_points_s_rescale[i-1]+delta_s);
                            }
                        }
                        tk::spline s_x_local, s_y_local;
                        s_x_local.set_points(path_points_s_rescale,path_points_x);
                        s_y_local.set_points(path_points_s_rescale,path_points_y);
                        for(int i=0;i<path_points_s.size();i++) {
                            next_s_vals.push_back(path_points_s[i]);
                            next_d_vals.push_back(path_points_d[i]);
                            next_x_vals.push_back(s_x_local(path_points_s_rescale[i]));
                            next_y_vals.push_back(s_y_local(path_points_s_rescale[i]));
                        }

                    }

                    // End
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
















































































