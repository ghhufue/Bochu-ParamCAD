#include "ui/panels/PointCloudPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"

#include <QVBoxLayout>

PointCloudPanel::PointCloudPanel(QWidget* parent)
    : QWidget(parent), m_relationsPanel(new PointCloudRelationsPanel(this))
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_relationsPanel);
}
