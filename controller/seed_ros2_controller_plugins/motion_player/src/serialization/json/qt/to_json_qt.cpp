#include "to_json_qt.hpp"
#include "fileio_qt_tools.hpp"
#include <QPixmap>

picojson::value toJson(const QPixmap& value) {
    std::string str = pixmapToBase64(value);
    return picojson::value(str);
}

picojson::value toJson(const QImage& value) {
    std::string str = QImageToBase64(value);
    return picojson::value(str);
}

picojson::value toJson(const QByteArray& value){
    return picojson::value(value.toBase64().toStdString());
}
