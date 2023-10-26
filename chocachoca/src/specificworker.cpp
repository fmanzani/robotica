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

void SpecificWorker::compute()
{
    try
    {
        auto ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
        //qInfo() <<ldata.points.size();
        const auto &points = ldata.points;
        if(points.empty()) return;

        //Dibujamos los puntos
        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) {return p.z < 2000;});
        draw_lidar(filtered_points, viewer);

        switch(estado)
        {
            case Estado::IDLE:
                stop();
                break;
            case Estado::FOLLOW_WALL:
                follow_wall(const_cast<RoboCompLidar3D::TPoints &>(points));
                break;
            case Estado::STRAIGHT_LINE:
                straight_line(const_cast<RoboCompLidar3D::TPoints &>(points));
                break;
            case Estado::SPIRAL:
                spiral(const_cast<RoboCompLidar3D::TPoints &>(points));
                break;
        }

    }
    catch(const Ice::Exception &e){
        std::cout << "Error";
    }
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
    }
    borrar.clear();

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
    /*
    int offset = points.size()/2-points.size()/5;
    RoboCompLidar3D::TPoint punto = points[offset];
    RoboCompLidar3D::TPoint robot = points[0];
    double con = atan2(punto.y, punto.x);
    linea = viewer->scene.addLine(robot.x, robot.y, punto.x*con, punto.y*con, QPen(QColor("red"), 40));
    */
}

std::tuple<SpecificWorker::Estado, SpecificWorker::robotSpeed> SpecificWorker::straight_line(RoboCompLidar3D::TPoints &points) {

    int offset = points.size()/2-points.size()/5;
    auto min_elem = std::min_element(points.begin()+offset, points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
    robotSpeed rs;

    const float MIN_DISTANCE = 1000;
    qInfo() << std::hypot(min_elem->x, min_elem->y);
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE){
        try {
            // Se para el robot y gira
            omnirobot_proxy->setSpeedBase(0,0,0.5);
            rs = robotSpeed{0,0,0.5};
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }else{
        try {
            //enciende el robot
            omnirobot_proxy->setSpeedBase(1000/1000.f, 0, 0);
            rs = robotSpeed{1000/1000.f,0,0};
        } catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }

    return std::make_tuple(Estado::STRAIGHT_LINE, rs);
}

std::tuple<SpecificWorker::Estado, SpecificWorker::robotSpeed> SpecificWorker::stop() {



}

std::tuple<SpecificWorker::Estado, SpecificWorker::robotSpeed> SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &points) {

    int offset = points.size()/2-points.size()/5;
    auto min_elem = std::min_element(points.begin()+offset, points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
    robotSpeed rs;

    const float MIN_DISTANCE_Y = 500;
    const float MIN_DISTANCE_X = 550;
    qInfo() << std::hypot(min_elem->x, min_elem->y);
    RoboCompLidar3D::TPoint punto_inicio;
    qInfo() << min_elem->x << min_elem->y << min_elem->theta;

    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE_Y){
             try {
                 omnirobot_proxy->setSpeedBase(0, M_PI/4, 1);
                 rs = robotSpeed{0, M_PI/4, 1};
             }
             catch (const Ice::Exception &e) {
                 std::cout << "Error reading from Camera" << e << std::endl;
             }
    }else{
        try {
            //enciende el robot
            omnirobot_proxy->setSpeedBase(1000/1000.f, 0, 0);
            rs = robotSpeed{1000/1000.f,0,0};

        } catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }
    if(min_elem->x > MIN_DISTANCE_X){
        omnirobot_proxy->setSpeedBase(0, -M_PI/4, -1);
        rs = robotSpeed{0, -M_PI/4, -1};
    }


    return std::make_tuple(Estado::FOLLOW_WALL, rs);

}

std::tuple<SpecificWorker::Estado, SpecificWorker::robotSpeed> SpecificWorker::spiral(RoboCompLidar3D::TPoints &points) {

    int offset = points.size()/2-points.size()/5;
    auto min_elem = std::min_element(points.begin()+offset, points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});

    qInfo() << min_elem->x << " " << min_elem->y;

    omnirobot_proxy->setSpeedBase(0, M_PI/(10-t), rot);
    if(min_elem->x > 2400 && !cambiar) {
        rot *= 0.99;
        t += 0.05;
        cambiar = true;
    }
    if(min_elem->x < 2400 && cambiar){
        cambiar = false;
    }
    qInfo() << "t: " << t << "Giro: " << M_PI/(8-t) << "r: " << rot;

    rs = robotSpeed{0, M_PI/2, 1};
    return std::make_tuple(Estado::SPIRAL, rs);
}



/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

