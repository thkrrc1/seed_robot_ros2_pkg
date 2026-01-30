#pragma once

#include "common.hpp"
#include "type_traits.hpp"
#include <picojson.h>

class QPixmap;
class QByteArray;
class QImage;

void fromJson(picojson::value json, QPixmap &value);
void fromJson(picojson::value json, QByteArray &value);
void fromJson(picojson::value json, QImage &value);

