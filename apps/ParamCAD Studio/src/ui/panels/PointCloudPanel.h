#pragma once
#include <QWidget>

class PointCloudRelationsPanel;

class PointCloudPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit PointCloudPanel(QWidget* parent = nullptr);
    PointCloudRelationsPanel* relationsPanel() const { return m_relationsPanel; }

private:
    PointCloudRelationsPanel* m_relationsPanel;
};
