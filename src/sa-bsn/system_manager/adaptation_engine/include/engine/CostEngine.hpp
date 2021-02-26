#ifndef COSTENGINE_HPP
#define COSTENGINE_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <algorithm>

#include "ros/ros.h"
#include "ros/package.h"

#include "libbsn/goalmodel/Node.hpp"
#include "libbsn/goalmodel/Goal.hpp"
#include "libbsn/goalmodel/Task.hpp"
#include "libbsn/goalmodel/Property.hpp"
#include "libbsn/goalmodel/LeafTask.hpp"
#include "libbsn/goalmodel/Context.hpp"
#include "libbsn/goalmodel/GoalTree.hpp"
#include "libbsn/model/Formula.hpp"
#include "libbsn/utils/utils.hpp"

#include "lepton/Lepton.h"

#include "archlib/DataAccessRequest.h"
#include "archlib/Strategy.h"
#include "archlib/Exception.h"
#include "archlib/EnergyStatus.h"
#include "archlib/ROSComponent.hpp"
#include "archlib/EngineRequest.h"

#include "engine/Engine.hpp"

class CostEngine : public Engine {
	
	public: 
		CostEngine(int &argc, char **argv, std::string name);
    	virtual ~CostEngine();

    private:
      	CostEngine(const CostEngine &);
    	CostEngine &operator=(const CostEngine &);

  	public:
		void monitor();
    	void analyze();
		void plan();
    	void execute();
};

#endif 