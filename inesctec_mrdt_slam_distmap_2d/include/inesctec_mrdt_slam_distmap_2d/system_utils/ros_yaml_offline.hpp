#pragma once

#include <exception>

// YAML
#include <yaml-cpp/yaml.h>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace system_utils
{

/**
 * @brief get a parameter from a YAML file using the variable name as the key to
 *        access the correct field in the YAML :: Node
 *        (throws std::invalid_argument exception upon incorrect variable name)
 * @tparam T variable type (must be compatible with the ones supported by the
 *           yaml-cpp library - see their documentation!)
 * @param[in] config YAML node
 * @param[in] variable_name variable name in the YAML node
 * @return T parameter value
 */
template <typename T>
T getParam(const YAML::Node& config, std::string variable_name)
{
  if (config[variable_name])
  {
    return config[variable_name].as<T>();
  }
  else
  {
    throw std::invalid_argument("Parameter " + variable_name + " not found...");
  }
}  // template <typename T> T getParam(const YAML::Node&, std::string)

/**
 * @brief get a parameter from a YAML file using the variable name as the key to
 *        access the correct field in the YAML :: Node
 *        (throws std::invalid_argument exception upon incorrect variable name)
 * @tparam T variable type (must be compatible with the ones supported by the
 *           yaml-cpp library - see their documentation!)
 * @param[in] config YAML node
 * @param[in] variable_name variable name in the YAML node
 * @param[out] variable parameter variable
 */
template <typename T>
void getParam(const YAML::Node& config, std::string variable_name, T& variable)
{
  if (config[variable_name])
  {
    variable = config[variable_name].as<T>();
  }
  else
  {
    throw std::invalid_argument("Parameter " + variable_name + " not found...");
  }
}  // template <typename T> void getParam(const YAML::Node&, std::string, T&)

/**
 * @brief get a parameter from a YAML file using the variable name as the key to
 *        access the correct field in the YAML :: Node, if NOT FOUND, see a
 *        default value to the parameter variable
 * @tparam T variable type (must be compatible with the ones supported by the
 *           yaml-cpp library - see their documentation!)
 * @param[in] config YAML node
 * @param[in] variable_name variable name in the YAML node
 * @param[out] variable parameter variable
 * @param[in] default_value default value for the parameter variable (if not set
 *                          / defined in the YAML file)
 */
template <typename T>
void getParam(const YAML::Node& config, std::string variable_name, T& variable,
              const T& default_value)
{
  if (config[variable_name])
  {
    variable = config[variable_name].as<T>();
  }
  else
  {
    variable = default_value;
  }
}  // template <typename T> void getParam(const YAML::Node&, std::string, T&,
   // const T&)

/**
 * @brief query if the YAML node has defined a certain variable
 * @tparam T variable type (must be compatible with the ones supported by the
 *           yaml-cpp library - see their documentation!)
 * @param[in] config YAML node
 * @param[in] variable_name variable name in the YAML node
 * @return true variable exists in the YAML node
 * @return false otherwise
 */
bool hasParam(const YAML::Node& config, std::string variable_name)
{
  return bool(config[variable_name]);
}  // void hasParam(const YAML::Node&, std::string)

}  // namespace system_utils

}  // namespace inesctec_mrdt_slam_distmap_2d
