/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rqt_image_view/ratio_layouted_frame.h>

#include <assert.h>
#include <ros/ros.h>

namespace rqt_image_view {

RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WFlags flags)
 : QFrame(), offset_(0, 0), stretch_(false), forceRescale_(true)
{
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()), Qt::QueuedConnection);
}

RatioLayoutedFrame::~RatioLayoutedFrame()
{
}

const QImage& RatioLayoutedFrame::getImage() const
{
  return qimage_;
}

void RatioLayoutedFrame::setImage(const QImage& image)//, QMutex* image_mutex)
{
  qimage_mutex_.lock();
    forceRescale_ = true;
  qimage_ = image.copy();

    if( qimage_.isNull() )
        qimageScaled_ = qimage_;

  qimage_mutex_.unlock();

  emit delayed_update();
}

void RatioLayoutedFrame::rescale()
{
    if( qimage_.isNull() )
        return;

    if( stretch_ == false && forceRescale_ )
    { 
        float cw = width(), ch = height();
        float pw = qimage_.width(), ph = qimage_.height();

        if (pw > cw && ph > ch && pw/cw > ph/ch ||  //both width and high are bigger, ratio at high is bigger or
            pw > cw && ph <= ch ||                  //only the width is bigger or
            pw < cw && ph < ch && cw/pw < ch/ph )   //both width and height is smaller, ratio at width is smaller
        {
            qimageScaled_ = qimage_.scaledToWidth(cw, Qt::FastTransformation);
        }
        else if (pw > cw && ph > ch && pw/cw <= ph/ch || //both width and high are bigger, ratio at width is bigger or
                 ph > ch && pw <= cw ||                  //only the height is bigger or
                 pw < cw && ph < ch && cw/pw > ch/ph)    //both width and height is smaller, ratio at height is smaller
        {
            qimageScaled_ = qimage_.scaledToHeight(ch, Qt::FastTransformation);
        }

        offset_.setX( (cw - qimageScaled_.width()) / 2 );
        offset_.setY( (ch - qimageScaled_.height()) / 2 );

        forceRescale_ = false;
    }
}

void RatioLayoutedFrame::resizeEvent(QResizeEvent * event)
{
    forceRescale_ = true;
    rescale();
}

void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize& size)
{
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMinimumSize(new_size);
  emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize& size)
{
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMaximumSize(new_size);
  emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameFixedSize(const QSize& size)
{
  setInnerFrameMinimumSize(size);
  setInnerFrameMaximumSize(size);
}

void RatioLayoutedFrame::toggleStretch()
{
    stretch_ = ! stretch_;
}

void RatioLayoutedFrame::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  qimage_mutex_.lock();
  if (!qimage_.isNull())
  {
    if( stretch_ )
         painter.drawImage(contentsRect(), qimage_);
    else {
         rescale();
         painter.drawImage(offset_, qimageScaled_);
    }
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
  qimage_mutex_.unlock();
}

int RatioLayoutedFrame::greatestCommonDivisor(int a, int b)
{
  if (b==0)
  {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

}
