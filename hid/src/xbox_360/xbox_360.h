////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals Human Iterface Device Package
//
// Link:      https://github.com/roadnarrows-robotics/peripherals
//
// ROS Node:  xbox_360
//
// File:      xbox_360.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS xbox_360 node class interface.
 *
 * \author Danial Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _XBOX_360_H
#define _XBOX_360_H

#include <string>
#include <map>

//
// Includes for boost libraries
//
#include <boost/bind.hpp>

//
// ROS
//
#include "ros/ros.h"

//
// ROS generated hid messages.
//
#include "hid/ConnStatus.h"
#include "hid/Controller360State.h"

//
// ROS generatated hid services.
//
#include "hid/Ping.h"
#include "hid/SetLED.h"
#include "hid/SetRumble.h"
#include "hid/RumbleCmd.h"

//
// RoadNarrows embedded
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/hid/HIDXbox360.h"

namespace hid
{
  /*!
   * \brief The class embodiment of the xbox_360 ROS node.
   */
  class Xbox360
  {
  public:
    /*! map of ROS services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh        Bound node handle.
     * \param hidXbox   Bound Xbox360 HID
     */
    Xbox360(ros::NodeHandle &nh, rnr::HIDXbox360 &hidXbox);

    /*!
     * \brief Destructor.
     */
    virtual ~Xbox360();

    /*!
     * \brief Advertise all services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Advertise all publishers.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void advertisePublishers(int nQueueDepth=10);

    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void subscribeToTopics(int nQueueDepth=10);

    /*!
     * \brief Publish.
     *
     * Call in main loop.
     */
    virtual void publish();

    /*!
     * \brief Force publish disconnected connection status message.
     */
    virtual void publishDisconnect();

    /*!
     * \brief Get bound node handle.
     *
     * \return Node handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    /*!
     * \brief Get bound embedded hid instance.
     *
     * \return Xbox360 HID instance.
     */
    rnr::HIDXbox360 &getHID()
    {
      return m_hidXbox;
    }

    /*!
     * \brief Query if HID is connected.
     *
     * \return Returns true if (physically) connected, else false.
     */
    bool isConnected()
    {
      return m_hidXbox.isConnected();
    }

    /*!
     * \brief Create and run USB update in thread.
     *
     * \param hz  Update Hertz.
     *
     * \return Returns 0 on success, \h_lt 0 on failure.
     */
    void run(float hz=30.0)
    {
      m_hidXbox.run();
    }

    /*!
     * \brief Stop and destroy USB update thread.
     *
     * \return Returns 0 on success, \h_lt 0 on failure.
     */
    void stop()
    {
      m_hidXbox.stop();
    }

  protected:
    ros::NodeHandle  &m_nh;       ///< the node handler bound to this instance
    rnr::HIDXbox360  &m_hidXbox;  ///< real-time, Xbox360 HID

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< pan-tilt control services
    MapPublishers     m_publishers;     ///< pan-tilt control publishers
    MapSubscriptions  m_subscriptions;  ///< pan-tilt control subscriptions

    // Messages for published data.
    hid::Controller360State m_msgXboxState;   ///< button state
    hid::ConnStatus         m_msgConnStatus;  ///< connection status


    //..........................................................................
    // Service callbacks
    //..........................................................................

    /*!
     * \brief Ping Xbox360 game controller.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool ping(hid::Ping::Request  &req,
              hid::Ping::Response &rsp);

    /*!
     * \brief Set Xbox360 game controller's LED pattern.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setLED(hid::SetLED::Request  &req,
                hid::SetLED::Response &rsp);

    /*!
     * \brief Set Xbox360 game controller's rumble motor speeds.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setRumble(hid::SetRumble::Request  &req,
                   hid::SetRumble::Response &rsp);


    //..........................................................................
    // Topic Publishers
    //..........................................................................

    /*!
     * \brief Publish Xbox360 button state.
     */
    void publishXboxState();

    /*!
     * \brief Publish Xbox360 connectivity status.
     */
    void publishConnStatus();


    //..........................................................................
    // Subscribed Topic Callbacks
    //..........................................................................

    /*!
     * \brief Execute rumble command.
     *
     * \param cmd   Command.
     */
    void execRumbleCmd(const hid::RumbleCmd &cmd);
  };

} // namespace hid


#endif // _XBOX_360_H
