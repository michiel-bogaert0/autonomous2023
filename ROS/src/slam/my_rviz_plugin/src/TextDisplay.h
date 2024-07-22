#ifndef TEXT_DISPLAY_H
#define TEXT_DISPLAY_H

#include <rviz/display.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz_plugin_tutorials
{

class TextDisplay : public rviz::Display
{
Q_OBJECT
public:
  TextDisplay();
  virtual ~TextDisplay();

protected:
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void updateText();
  void updatePosition();

private:
  rviz::VectorProperty* position_property_;
  rviz::StringProperty* text_property_;
  rviz::MovableText* text_;
  Ogre::SceneNode* scene_node_;
};

} // namespace rviz_plugin_tutorials

#endif // TEXT_DISPLAY_H
