/*
 * roiDrawingArea.h
 *
 *  Created on: Feb 20, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_ROIDRAWINGAREA_H_
#define RS_LOG_LEARN_SRC_ROIDRAWINGAREA_H_

#include <gtkmm/drawingarea.h>

namespace rs_log_learning
{

class roiDrawingArea: public Gtk::DrawingArea
{
public:
  roiDrawingArea();
  virtual ~roiDrawingArea();
  void exposeTest();

protected:
  bool on_expose_event(GdkEventExpose *event);
};

} /* namespace rs_log_learning */

#endif /* RS_LOG_LEARN_SRC_ROIDRAWINGAREA_H_ */
