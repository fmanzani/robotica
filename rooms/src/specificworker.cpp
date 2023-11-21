/*
 *    Copyright (C) 2023 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }



	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		//Inicializaciones personales
        viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
        viewer->add_robot(400, 480, 0, 100, QColor("Blue"));
        viewer->show();

        timer.start(Period);
	}

}

SpecificWorker::Lines SpecificWorker::extract_lines(RoboCompLidar3D::TPoints points){

    Lines l;
    long unsigned int i;
    for(i = 0; i < points.size(); i++){
        if(points[i].z > 200 && points[i].z < 500) {
            qInfo() << points[i].theta * 180/M_PI;
            l.low.push_back(points[i]);
        }else if(points[i].z > 600 && points[i].z < 800){
            l.mid.push_back(points[i]);
        }else if(points[i].z > 1000 && points[i].z < 1200){
            l.high.push_back(points[i]);
        }
    }

    return l;
}

void SpecificWorker::compute()
{
    try
    {
        auto ldata = lidar3d_proxy->getLidarData("helios", 0, 2*M_PI, 1);
        //qInfo() <<ldata.points.size();
        const auto &points = ldata.points;
        if(points.empty()) return;

        //Dibujamos los puntos
        //RoboCompLidar3D::TPoints filtered_points;

        SpecificWorker::Lines lines;
        lines = extract_lines(ldata.points);

        //std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) {return p.z < 2000;});
        draw_lidar(lines.mid, viewer);
        auto peaks = extract_peaks(lines);
        //draw_doors(peaks.low, viewer);
        auto doors = get_doors(peaks);
        draw_doors(doors, viewer);
    }
    catch(const Ice::Exception &e){
        std::cout << "Error";
    }
}

SpecificWorker::Lines SpecificWorker::extract_peaks(const SpecificWorker::Lines &lines){

    Lines peaks;
    const float THRES_PEAK = 1000;

    for(const auto &both: iter::sliding_window(lines.low, 2)){
        if(fabs(both[1].r - both[0].r) > THRES_PEAK){
            peaks.low.push_back(both[0]);
        }
    }for(const auto &both: iter::sliding_window(lines.mid, 2)){
        if(fabs(both[1].r - both[0].r) > THRES_PEAK){
            peaks.mid.push_back(both[0]);
        }
    }for(const auto &both: iter::sliding_window(lines.high, 2)){
        if(fabs(both[1].r - both[0].r) > THRES_PEAK){
            peaks.high.push_back(both[0]);
        }
    }
    return peaks;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    //Limpiamos el vector borrar
    for(auto &b: borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }    borrar.clear();

    //Pintamos los puntos alrededor del robot
    for(const auto &p: points){
        auto point = viewer->scene.addRect(-25, -25, 50, 50, QPen(QColor("blue")), QBrush(QColor("blue")));
        point->setPos(p.x, p.y);
        //qInfo() << p.x << p.y;
        borrar.push_back(point);
    }

    if(linea != nullptr){
        viewer->scene.removeItem(linea);
        linea = nullptr;
    }
}
void SpecificWorker::draw_doors(const SpecificWorker::Doors doors, AbstractGraphicViewer *viewer) {

    static std::vector<QGraphicsItem*> borrar;
    //Limpiamos el vector borrar
    for(auto &b: borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }    borrar.clear();

    for(const auto &d : doors) {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
        point->setPos(d.left.x, d.left.y);
        borrar.push_back(point);
        point = viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
        point->setPos(d.right.x, d.right.y);
        borrar.push_back(point);
        auto line = viewer->scene.addLine(d.left.x, d.left.y, d.right.x, d.right.y, QPen(QColor("green"), 50));
        borrar.push_back(line);
    }
    if(linea != nullptr){
        viewer->scene.removeItem(linea);
        linea = nullptr;
    }

}

SpecificWorker::Doors SpecificWorker::get_doors(const SpecificWorker::Lines &peaks) {

    Doors doors;

    auto dist = [](auto a, auto b){
        qInfo() << std::hypot(a.x-b.x, a.y-b.y);
        return std::hypot(a.x-b.x, a.y-b.y);
    };

    const float THRES_DOOR = 500;

    auto near_door = [&doors, dist, THRES_DOOR](auto d){
        for(auto &&old: doors){
            qInfo() << dist(old.left, d.left) << dist(old.right, d.right) << dist(old.right, d.left) << dist(old.left, d.right);
            if(dist(old.left, d.left) < THRES_DOOR or dist(old.right, d.right) < THRES_DOOR or dist(old.right, d.left) < THRES_DOOR or dist(old.left, d.right) < THRES_DOOR){
                return true;
            }
        }
        return false;
    };

    for(auto &par : peaks.mid | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(door)) {
                doors.emplace_back(Door{par[0], par[1]});
            }
        }
    }

    return doors;
}


/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

