#pragma once
#include <functional>

class QWidget;

void showHyperparameterSettings(QWidget* parent,const std::function<void()>& analysisSettingsChanged = {});
