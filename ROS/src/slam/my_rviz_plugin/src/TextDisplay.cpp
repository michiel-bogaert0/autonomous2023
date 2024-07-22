#include "TextDisplay.h"

namespace rviz_plugin_tutorials
{

TextDisplay::TextDisplay()
{
  position_property_ = new rviz::VectorProperty("Position", Ogre::Vector3::ZERO, "Position of the text in the 3D space.", this, SLOT(updatePosition()));
  text_property_ = new rviz::StringProperty("Text", "Hello, RViz!", "Text to display.", this, SLOT(updateText()));
}

TextDisplay::~TextDisplay()
{
  if (initialized())
  {
    scene_manager_->destroySceneNode(scene_node_);
  }
}

void TextDisplay::onInitialize()
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  text_ = new rviz::MovableText(text_property_->getStdString());
  text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  scene_node_->attachObject(text_);
  updatePosition();
  updateText();
}

void TextDisplay::onEnable()
{
  scene_node_->setVisible(true);
}

void TextDisplay::onDisable()
{
  scene_node_->setVisible(false);
}

void TextDisplay::updateText()
{
  text_->setCaption(text_property_->getStdString());
}

void TextDisplay::updatePosition()
{
  scene_node_->setPosition(position_property_->getVector());
}

} // namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::TextDisplay, rviz::Display)
