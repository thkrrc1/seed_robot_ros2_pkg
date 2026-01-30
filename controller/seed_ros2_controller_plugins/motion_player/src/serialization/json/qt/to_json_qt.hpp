#pragma once

#include "common.hpp"
#include <picojson.h>

class QPixmap;
class QByteArray;
class QImage;

picojson::value toJson(const QPixmap& value);
picojson::value toJson(const QByteArray& value);
picojson::value toJson(const QImage& value);
