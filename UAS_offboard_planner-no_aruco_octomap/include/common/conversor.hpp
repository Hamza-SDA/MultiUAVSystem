#ifndef _CONVERSOR_HPP_
#define _CONVERSOR_HPP_
/**
 * @file conversor.hpp
 * @author Juan Sandino (juan.sandino@hdr.qut.edu.au)
 * @brief
 * @version 0.3.0
 * @date 2021-04-16
 *
 * @copyright Copyright (c) 2020-2021 Juan Sandino, QUT.
 *
 */
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace conversor
{
  /**
   * @brief Parse ROS params from a YAML file.
   *
   * @tparam T Variable type.
   * @param node The ROS node handler.
   * @param key The variable key name.
   * @param out_value The assigned value.
   * @param def_value The default value if key cannot be read from YAML file.
   */
  template <typename T>
  void parseROSParam(const ros::NodeHandle &node, const std::string &key, T &out_value, const T &def_value)
  {
    T read_param;
    if (node.getParam(key, read_param))
    {
      out_value = read_param;
    }
    else
    {
      out_value = def_value;
      ROS_ERROR("Cannot read key '%s'", key.c_str());
      ROS_ERROR_STREAM("Setting value as: " << out_value);
    }
  }

} // namespace conversor

#endif /* _CONVERSOR_HPP_ */