/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging MoveIt! applications
*/

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QGroupBox>
#include <QSpinBox>

#include <sensor_msgs/Joy.h>

#include "moveit_panel.h"

namespace moveit_dashboard
{
MoveItPanel::MoveItPanel(QWidget* parent) : rviz::Panel(parent)
{
  // Create a push button
  btn_next_ = new QPushButton(this);
  btn_next_->setText("Next");
  connect(btn_next_, SIGNAL(clicked()), this, SLOT(moveNext()));

  // Create a push button
  btn_auto_ = new QPushButton(this);
  btn_auto_->setText("Auto");
  connect(btn_auto_, SIGNAL(clicked()), this, SLOT(moveAuto()));

  // Create a push button
  btn_full_auto_ = new QPushButton(this);
  btn_full_auto_->setText("Full Auto");
  connect(btn_full_auto_, SIGNAL(clicked()), this, SLOT(moveFullAuto()));

  // Create a push button
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop");
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(moveStop()));

  // Create a push button
  btn_reset_ = new QPushButton(this);
  btn_reset_->setText("Zero");
  connect(btn_reset_, SIGNAL(clicked()), this, SLOT(zeroRobot()));

  // Create a push button
  btn_bringup_ = new QPushButton(this);
  btn_bringup_->setText("Play");
  connect(btn_bringup_, SIGNAL(clicked()), this, SLOT(playTrajectory()));

  // Create a push button
  btn_home_ = new QPushButton(this);
  btn_home_->setText("Home");
  connect(btn_home_, SIGNAL(clicked()), this, SLOT(homeRobot()));

  // Create a push button
  btn_grip_ = new QPushButton(this);
  btn_grip_->setText("Stop");
  connect(btn_grip_, SIGNAL(clicked()), this, SLOT(stopTrajectory()));

  // Horizontal Layout
  QHBoxLayout* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_next_);
  hlayout1->addWidget(btn_auto_);
  hlayout1->addWidget(btn_full_auto_);
  hlayout1->addWidget(btn_stop_);

  // Horizontal Layout
  QHBoxLayout* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(new QLabel(QString("Robot:")));
  hlayout2->addWidget(btn_home_);
  hlayout2->addWidget(btn_reset_);
  hlayout2->addWidget(btn_bringup_);
  hlayout2->addWidget(btn_grip_);
  // hlayout2->addWidget(spin_box_);
  // hlayout2->addWidget(combo_mode_);

  // Horizontal Layout
  // QHBoxLayout* hlayout3 = new QHBoxLayout;

  this->setStyleSheet("QGroupBox {  border: 1px solid gray; padding-top: 0px; }");

  // Group box
  QGroupBox* group_box = new QGroupBox();
  group_box->setLayout(hlayout2);
  group_box->setFlat(false);

  // Verticle layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addWidget(group_box);
  // layout->addLayout( hlayout3 );
  setLayout(layout);

  joy_publisher_ = nh_.advertise<sensor_msgs::Joy>("/moveit_rviz_dashboard", 1);

  btn_next_->setEnabled(true);
  btn_auto_->setEnabled(true);
  btn_full_auto_->setEnabled(true);
}

void MoveItPanel::moveNext()
{
  ROS_INFO_STREAM_NAMED("moveit_dashboard", "Move to next step");

  sensor_msgs::Joy msg;
  msg.buttons.resize(9);
  msg.buttons[1] = 1;
  joy_publisher_.publish(msg);
}

void MoveItPanel::moveAuto()
{
  ROS_INFO_STREAM_NAMED("moveit_dashboard", "Running auto step");

  sensor_msgs::Joy msg;
  msg.buttons.resize(9);
  msg.buttons[2] = 1;
  joy_publisher_.publish(msg);
}

void MoveItPanel::moveFullAuto()
{
  ROS_INFO_STREAM_NAMED("moveit_dashboard", "Running full auto");

  sensor_msgs::Joy msg;
  msg.buttons.resize(9);
  msg.buttons[3] = 1;
  joy_publisher_.publish(msg);
}

void MoveItPanel::moveStop()
{
  ROS_INFO_STREAM_NAMED("moveit_dashboard", "Stopping");

  sensor_msgs::Joy msg;
  msg.buttons.resize(9);
  msg.buttons[4] = 1;
  joy_publisher_.publish(msg);
}

void MoveItPanel::zeroRobot()
{
  ROS_INFO_STREAM_NAMED("moveit_dashboard", "Zero Pose");

  sensor_msgs::Joy msg;
  msg.buttons.resize(9);
  msg.buttons[5] = 1;
  joy_publisher_.publish(msg);
}

void MoveItPanel::playTrajectory()
{
  ROS_INFO_STREAM_NAMED("moveit_dashboard", "Play trajectory");

  sensor_msgs::Joy msg;
  msg.buttons.resize(9);
  msg.buttons[6] = 1;
  joy_publisher_.publish(msg);
}

void MoveItPanel::stopTrajectory()
{
  ROS_INFO_STREAM_NAMED("moveit_dashboard", "Stopping trajectory");

  sensor_msgs::Joy msg;
  msg.buttons.resize(9);
  msg.buttons[7] = 1;
  joy_publisher_.publish(msg);
}

void MoveItPanel::homeRobot()
{
  ROS_INFO_STREAM_NAMED("moveit_dashboard", "Home Pose");

  sensor_msgs::Joy msg;
  msg.buttons.resize(9);
  msg.buttons[8] = 1;
  joy_publisher_.publish(msg);
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void MoveItPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}
// Load all configuration data for this panel from the given Config object.
void MoveItPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace moveit_dashboard

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_dashboard::MoveItPanel, rviz::Panel)
