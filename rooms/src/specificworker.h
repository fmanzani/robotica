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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <ranges>
#include <cmath>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	int startup_check();
	void initialize(int period);


private:
    struct Lines {
        RoboCompLidar3D::TPoints high, mid, low;
    };
    struct Door {
        RoboCompLidar3D::TPoint left, right;
    };
    using Doors = std::vector<Door>;
    SpecificWorker::Lines extract_peaks(const SpecificWorker::Lines &lines);
    Lines extract_lines(RoboCompLidar3D::TPoints points);
	bool startup_check_flag;
    AbstractGraphicViewer* viewer;
    QGraphicsLineItem* linea = nullptr;
    void draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer);
    void draw_doors(const SpecificWorker::Doors doors, AbstractGraphicViewer *viewer);
    Doors get_doors(const Lines &peaks);
};

#endif
