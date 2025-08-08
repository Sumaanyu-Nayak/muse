#include "state_estimator/state_estimator_node.hpp"

namespace state_estimator {

state_estimator_node::~state_estimator_node() {
    shutdown();
}

void state_estimator_node::shutdown() {
	RCLCPP_INFO(this->get_logger(), "SE node: Shutting node down...");
	for (auto const&plugin:loaded_plugins) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down " << plugin->getName() << "...");
		plugin->shutdown();
		RCLCPP_INFO(this->get_logger(), "Done");
	}
	RCLCPP_INFO(this->get_logger(), "State Estimator shut down.");
}

state_estimator_node::state_estimator_node(const rclcpp::NodeOptions & options) : rclcpp::Node("state_estimator", options), plugin_loader("state_estimator", "state_estimator_plugins::PluginBase") {
   // robot = Anymal::create();

	// Declare parameters
	this->declare_parameter("plugin_blacklist", std::vector<std::string>());
	this->declare_parameter("plugin_whitelist", std::vector<std::string>());

	std::vector<std::string> blacklist{}, whitelist{};

	getBlacklist(blacklist);
	getWhitelist(whitelist);

	//Assume all plugins are blacklisted
	if(blacklist.empty() and !whitelist.empty())
		blacklist.emplace_back("*");

	for (auto &name : plugin_loader.getDeclaredClasses()) {
		add_plugin(name, blacklist, whitelist);
	}

  get_active_estimators_srv_ = this->create_service<state_estimator_msgs::srv::GetActiveEstimators>("getActiveEstimators", std::bind(&state_estimator_node::getActiveEstimators, this, std::placeholders::_1, std::placeholders::_2));
  get_blacklist_srv_ = this->create_service<state_estimator_msgs::srv::GetBlacklist>("getBlacklist", std::bind(&state_estimator_node::getBlacklistService, this, std::placeholders::_1, std::placeholders::_2));
  get_whitelist_srv_ = this->create_service<state_estimator_msgs::srv::GetWhitelist>("getWhitelist", std::bind(&state_estimator_node::getWhitelistService, this, std::placeholders::_1, std::placeholders::_2));
  list_all_estimators_srv_ = this->create_service<state_estimator_msgs::srv::ListAllEstimators>("listAllEstimators", std::bind(&state_estimator_node::listAllEstimators, this, std::placeholders::_1, std::placeholders::_2));
  pause_estimator_srv_ = this->create_service<state_estimator_msgs::srv::PauseEstimator>("pauseEstimator", std::bind(&state_estimator_node::pauseEstimator, this, std::placeholders::_1, std::placeholders::_2));
  reset_estimator_srv_ = this->create_service<state_estimator_msgs::srv::ResetEstimator>("resetEstimator", std::bind(&state_estimator_node::resetEstimator, this, std::placeholders::_1, std::placeholders::_2));
  restart_estimator_srv_ = this->create_service<state_estimator_msgs::srv::RestartEstimator>("restartEstimator", std::bind(&state_estimator_node::restartEstimator, this, std::placeholders::_1, std::placeholders::_2));
  resume_estimator_srv_ = this->create_service<state_estimator_msgs::srv::ResumeEstimator>("resumeEstimator", std::bind(&state_estimator_node::resumeEstimator, this, std::placeholders::_1, std::placeholders::_2));
  start_estimator_srv_ = this->create_service<state_estimator_msgs::srv::StartEstimator>("startEstimator", std::bind(&state_estimator_node::startEstimator, this, std::placeholders::_1, std::placeholders::_2));
  stop_estimator_srv_ = this->create_service<state_estimator_msgs::srv::StopEstimator>("stopEstimator", std::bind(&state_estimator_node::stopEstimator, this, std::placeholders::_1, std::placeholders::_2));
  get_estimator_description_srv_ = this->create_service<state_estimator_msgs::srv::GetEstimatorDescription>("getEstimatorDescription", std::bind(&state_estimator_node::getEstimatorDescription, this, std::placeholders::_1, std::placeholders::_2));
}

bool state_estimator_node::add_plugin(std::string &name) {
    std::vector<std::string> blacklist{}, whitelist{};
    if (!getBlacklist(blacklist)) return false;
    if (!getWhitelist(whitelist)) return false;

    //Assume all plugins are blacklisted
    if(blacklist.empty() and !whitelist.empty())
	    blacklist.emplace_back("*");
    
    return add_plugin(name,blacklist,whitelist);
}

bool state_estimator_node::add_plugin(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist) {
	if (is_blacklisted(pl_name,blacklist,whitelist)) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << pl_name << " blacklisted");
		return false;
	}

	try {
		auto plugin = plugin_loader.createUniqueInstance(pl_name);
		RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << pl_name << " loaded");
		plugin->initialize(shared_from_this(), nullptr);
		RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << pl_name << " initialized");
		loaded_plugins.push_back(std::move(plugin));
		return true;
	} catch (pluginlib::PluginlibException &ex) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Plugin " << pl_name << " load exception: " << ex.what());
	} catch (std::exception &ex) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Plugin " << pl_name << " load exception: " << ex.what());
	} catch (...) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Plugin " << pl_name << " load exception.");
	}
	return false;
}

inline bool state_estimator_node::getBlacklist(std::vector<std::string> &blacklist) {
	try {
		auto param = this->get_parameter("plugin_blacklist");
		blacklist = param.get_value<std::vector<std::string>>();
	} catch (const std::exception& e) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get blacklist parameter: " << e.what());
		return false;
	}       
	return true;  
}

inline bool state_estimator_node::getWhitelist(std::vector<std::string> &whitelist) {
	try {
		auto param = this->get_parameter("plugin_whitelist");
		whitelist = param.get_value<std::vector<std::string>>();
	} catch (const std::exception& e) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get whitelist parameter: " << e.what());
		return false;
	}    
	return true;   
}

    /**
    * @brief Checks that plugin blacklisted
    *  1. if blacklist and whitelist is empty: load all
    *  2. if blacklist is empty and whitelist non empty: assume blacklist is ["*"]
    *  3. if blacklist non empty: usual blacklist behavior
    *  4. if whitelist non empty: override blacklist
    */
bool state_estimator_node::is_blacklisted(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist)
{
    for (auto &bl_pattern : blacklist) {
      if (pattern_match(bl_pattern, pl_name)) {
        for (auto &wl_pattern : whitelist) {
          if (pattern_match(wl_pattern, pl_name))
            return false;
        }
        return true;
      }
    }
    return false;
}



bool state_estimator_node::pattern_match(std::string &pattern, std::string &pl_name)
{
    int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
    if (cmp == 0)
      return true;
    else if (cmp != FNM_NOMATCH) {
      // never see that, i think that it is fatal error.
      RCLCPP_FATAL(this->get_logger(), "Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
          pattern.c_str(), pl_name.c_str(), cmp);
      rclcpp::shutdown();
    }

    return false;
}


//**************************************************
//Services
//**************************************************

void state_estimator_node::getActiveEstimators(const std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Request> req, std::shared_ptr<state_estimator_msgs::srv::GetActiveEstimators::Response> res) {
    for (auto const&plugin:loaded_plugins) {
	res->names.push_back(plugin->getName());
    }
}

void state_estimator_node::getBlacklistService(const std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Request> req, std::shared_ptr<state_estimator_msgs::srv::GetBlacklist::Response> res) {
    std::vector<std::string> blacklist{};
    if (!getBlacklist(blacklist)) return;
    for (auto const&plugin:blacklist) {
	res->names.push_back(plugin);
    }
}

void state_estimator_node::getWhitelistService(const std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Request> req, std::shared_ptr<state_estimator_msgs::srv::GetWhitelist::Response> res) {
    std::vector<std::string> whitelist{};
    if (!getWhitelist(whitelist)) return;
    for (auto const&plugin:whitelist) {
	res->names.push_back(plugin);
    }
}

void state_estimator_node::getEstimatorDescription(const std::shared_ptr<state_estimator_msgs::srv::GetEstimatorDescription::Request> req, std::shared_ptr<state_estimator_msgs::srv::GetEstimatorDescription::Response> res) {
    for (auto const&plugin:loaded_plugins) {
	if(plugin->getName().compare(req->name)==0)  {
	    res->description = plugin->getDescription();
	    res->success = true;
	    return;
	}
    }
    RCLCPP_WARN_STREAM(this->get_logger(), "Plugin " << req->name << " not loaded.");
    res->success = false;
}

void state_estimator_node::listAllEstimators(const std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Request> req, std::shared_ptr<state_estimator_msgs::srv::ListAllEstimators::Response> res) {
    for (auto &name : plugin_loader.getDeclaredClasses()) {
	    res->names.push_back(name);
    }
}

void state_estimator_node::pauseEstimator(const std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Request> req, std::shared_ptr<state_estimator_msgs::srv::PauseEstimator::Response> res) {
    for (auto const&plugin:loaded_plugins) {
	if(plugin->getName().compare(req->name)==0)  {
	    if (plugin->isPaused()) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << req->name << " already paused.");
		res->success=false;
		return;	
	    }
	    RCLCPP_INFO_STREAM(this->get_logger(), "Pausing " << req->name << "...");
	    plugin->pause();
	    RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << req->name << " paused.");
	    res->success = true;
	    return;
	}
    }
    RCLCPP_WARN_STREAM(this->get_logger(), "Plugin " << req->name << " not loaded.");
    res->success = false;
}

void state_estimator_node::resetEstimator(const std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Request> req, std::shared_ptr<state_estimator_msgs::srv::ResetEstimator::Response> res) {
    for (auto const&plugin:loaded_plugins) {
	if(plugin->getName().compare(req->name)==0)  {
	    RCLCPP_INFO_STREAM(this->get_logger(), "Reseting " << req->name << "...");
	    plugin->reset();
	    RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << req->name << " reset.");
	    res->success = true;
	    return;
	}
    }
    RCLCPP_WARN_STREAM(this->get_logger(), "Plugin " << req->name << " not loaded.");
    res->success = false;
}

void state_estimator_node::restartEstimator(const std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Request> req, std::shared_ptr<state_estimator_msgs::srv::RestartEstimator::Response> res) {
    for (auto const&plugin:loaded_plugins) {
	if(plugin->getName().compare(req->name)==0)  {
	    RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down " << req->name << "...");
	    plugin->shutdown();
	    RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << req->name << " shut down");
	    std::string plugin_name = req->name;
	    res->success = add_plugin(plugin_name);
	    return;
	}
    }
    RCLCPP_WARN_STREAM(this->get_logger(), "Plugin " << req->name << " not loaded.");
    res->success = false;
}

void state_estimator_node::resumeEstimator(const std::shared_ptr<state_estimator_msgs::srv::ResumeEstimator::Request> req, std::shared_ptr<state_estimator_msgs::srv::ResumeEstimator::Response> res) {
    for (auto const&plugin:loaded_plugins) {
	if(plugin->getName().compare(req->name)==0)  {
	    if (!plugin->isPaused()) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << req->name << " is not paused.");
		res->success=false;
		return;	
	    }
	    RCLCPP_INFO_STREAM(this->get_logger(), "Resuming " << req->name << "...");
	    plugin->resume();
	    RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << req->name << " resumed.");
	    res->success = true;
	    return;
	}
    }
    RCLCPP_WARN_STREAM(this->get_logger(), "Plugin " << req->name << " not loaded.");
    res->success = false;
}

void state_estimator_node::startEstimator(const std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Request> req, std::shared_ptr<state_estimator_msgs::srv::StartEstimator::Response> res) {
    std::string plugin_name = req->name;
    res->success = add_plugin(plugin_name);
}

void state_estimator_node::stopEstimator(const std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Request> req, std::shared_ptr<state_estimator_msgs::srv::StopEstimator::Response> res) {
    for (auto const&plugin:loaded_plugins) {
        if(plugin->getName().compare(req->name)==0)  {
            if (!plugin->isRunning()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << req->name << " is not running.");
            res->success=false;
            return;	
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down " << req->name << "...");
            plugin->shutdown();
            RCLCPP_INFO_STREAM(this->get_logger(), "Plugin " << req->name << " shut down");
            res->success=true;
            return;
        }
    }
    RCLCPP_WARN_STREAM(this->get_logger(), "Plugin " << req->name << " not loaded.");
    res->success=false;
}

} //namespace state_estimator

