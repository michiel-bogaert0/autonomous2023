#ifndef PathWithIds_DISPLAY_H
#define PathWithIds_DISPLAY_H

#ifndef Q_MOC_RUN

#include <nav_msgs/Path.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/line.h>
#include <ugr_msgs/PathWithIds.h>
#endif

namespace rviz_path_with_ids {

class PathWithIdsDisplay
    : public rviz::MessageFilterDisplay<ugr_msgs::PathWithIds> {
  Q_OBJECT
public:
  PathWithIdsDisplay();
  virtual ~PathWithIdsDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private:
  void clearDisplay();
  void processMessage(const ugr_msgs::PathWithIds::ConstPtr &msg);
  std::vector<rviz::Line *> visuals_;
};
} // end namespace rviz_path_with_ids
#endif