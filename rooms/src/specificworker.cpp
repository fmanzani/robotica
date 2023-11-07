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
            case Estado::FOLLOW_WALL:
                estado = follow_wall(const_cast<RoboCompLidar3D::TPoints &>(points));
                break;
            case Estado::STRAIGHT_LINE:
                estado = straight_line(const_cast<RoboCompLidar3D::TPoints &>(points));
                break;
            case Estado::SPIRAL:
                estado = spiral(const_cast<RoboCompLidar3D::TPoints &>(points));
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
}

SpecificWorker::Estado SpecificWorker::straight_line(RoboCompLidar3D::TPoints &points) {

    int offset = points.size()/2-points.size()/5;
    auto min_elem = std::min_element(points.begin()+offset, points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
    float rotacion = 3;

    const float MIN_DISTANCE = 600;
    qInfo() << std::hypot(min_elem->x, min_elem->y);
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE){
        try {
            if(giro > 50) {
                omnirobot_proxy->setSpeedBase(0, 0, rotacion);
            }else{
                omnirobot_proxy->setSpeedBase(0, 0, -rotacion);
            }
            if(giro > 100){
                giro = 0;
            }
            giro++;
            giros++;
            qInfo() << "Giros: " << giros;
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }else{
        try {
            omnirobot_proxy->setSpeedBase(2000/1000.f, 0, 0);
        } catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }
    if(giros > 35 && !spiralSL){
        if(std::hypot(min_elem->x, min_elem->y) > 1800){
            spiralSL = true;
            return Estado::SPIRAL;
        }
    }
    if(giros > 60) {
        giros = 1;
        int randomNumber = rand() % 100 + 1;
        qInfo() << "Espiral: " << randomNumber;
        if(randomNumber < 35){
            return Estado::SPIRAL;
        }else{
            return Estado::FOLLOW_WALL;
        }

    }
    comprobarBloqueo(*min_elem);
    return Estado::STRAIGHT_LINE;
}

bool checkRandomNumber() {
    int randomNumber = rand() % 100 + 1; // Genera un número aleatorio entre 1 y 1000
    qInfo() << "Random: " << randomNumber;
    return (randomNumber >= 10 && randomNumber <= 20);
}

SpecificWorker::Estado SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &points) {

    int offset = points.size()/2-points.size()/5;
    auto min_elem = std::min_element(points.begin()+offset, points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});

    qInfo() << "x: "<< min_elem->x << "y: " <<min_elem->y << "Valor: " << std::hypot(min_elem->x, min_elem->y);;

    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE_Y){
        try {
            omnirobot_proxy->setSpeedBase(1, M_PI/4, 3);
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }else{
        try {
            //enciende el robot
            omnirobot_proxy->setSpeedBase(2000/1000.f, 0, 0);

        } catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }
    if(min_elem->x > MIN_DISTANCE_X){
        omnirobot_proxy->setSpeedBase(0, -M_PI/4, -1.2);
    }
    MIN_DISTANCE_Y += 0.7;
    MIN_DISTANCE_X += 0.7;
    qInfo() << "DISTANCIAS: "<<MIN_DISTANCE_Y << " " << MIN_DISTANCE_X;

    srand(time(0));

    if(pasarEstado > 200){
        if(MIN_DISTANCE_Y < 1100) {
            if (checkRandomNumber()) {
                pasarEstado = 0;
                return Estado::STRAIGHT_LINE;
            }
        }else{
            int randomNumber = rand() % 100 + 1;
            if(randomNumber < 40){
                return Estado::SPIRAL;
            }else{
                return Estado::STRAIGHT_LINE;
            }
        }
    }else{
        pasarEstado++;
    }


    comprobarBloqueo(*min_elem);
    return Estado::FOLLOW_WALL;
}

void SpecificWorker::comprobarBloqueo(RoboCompLidar3D::TPoint &min_elem){
    if(min_elem.x != punto.x && min_elem.y != punto.y){
        punto.x = min_elem.x;
        punto.y = min_elem.y;
        repeticion = 0;
    }else{
        repeticion++;
        qInfo() << "Repeticion: " << repeticion;
    }
    if(repeticion > 4){
        for(int i = 0; i < 10; i++){
            omnirobot_proxy->setSpeedBase(-2, 0, 0);
        }
    }
}

SpecificWorker::Estado SpecificWorker::spiral(RoboCompLidar3D::TPoints &points) {

    int offset = points.size() / 2 - points.size() / 5;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b) {
                                         return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);
                                     });

    qInfo() << "Entro en SPIRAL";
    Estado estado = Estado::SPIRAL;

    if(!acabar) {
        qInfo() << min_elem->x << " " << min_elem->y << "Valor: " << std::hypot(min_elem->x, min_elem->y);

        omnirobot_proxy->setSpeedBase(0, M_PI / (10 - t), rot);
        rot *= 0.999;
        t += 0.05;

        qInfo() << "t: " << t << "Giro: " << M_PI / (8 - t) << "r: " << rot;

        if (t > 8.2) {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            acabar = true;
        }
        if(abs(min_elem->x) < 50){
            //omnirobot_proxy->setSpeedBase(-2, 2, 1.57);
            //acabar = true;
        }
    }else{
        qInfo() << "Valor de  Y: " << min_elem->y;
        t = 2;
        rot = 2;
        estado = Estado::FOLLOW_WALL;
        acabar = false;
    }

    return estado;
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

