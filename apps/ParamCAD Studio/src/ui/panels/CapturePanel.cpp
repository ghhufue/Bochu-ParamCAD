#include "ui/panels/CapturePanel.h"

#include "core/OrbitCamera.h"

#include <QDoubleSpinBox>
#include <QComboBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QScrollArea>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

CapturePanel::CapturePanel(QWidget* parent)
    : QWidget(parent)
{
    m_directoryLabel = new QLineEdit(this);
    m_directoryLabel->setObjectName(QStringLiteral("captureBaseDir"));
    m_directoryLabel->setReadOnly(true);

    m_browseButton = new QPushButton(tr("浏览…"), this);
    m_browseButton->setObjectName(QStringLiteral("captureBrowseButton"));

    auto* directoryRow = new QWidget(this);
    auto* directoryLayout = new QHBoxLayout(directoryRow);
    directoryLayout->setContentsMargins(0, 0, 0, 0);
    directoryLayout->setSpacing(4);
    directoryLayout->addWidget(m_directoryLabel, 1);
    directoryLayout->addWidget(m_browseButton);

    auto* outputGroup = new QGroupBox(tr("输出"), this);
    auto* outputForm = new QFormLayout(outputGroup);
    outputForm->addRow(tr("基础目录"), directoryRow);

    m_distance = new QDoubleSpinBox(this);
    m_distance->setObjectName(QStringLiteral("captureDistance"));
    m_distance->setRange(0.001, 100000.0);
    m_distance->setDecimals(3);
    m_distance->setSingleStep(1.0);
    m_distance->setValue(OrbitParameters{}.distance);

    auto* orbitGroup = new QGroupBox(tr("球面拍摄"), this);
    auto* orbitForm = new QFormLayout(orbitGroup);
    orbitForm->addRow(tr("距离"), m_distance);
    auto* gridSummary = new QLabel(
        tr("共 62 个视角：经度每 30°，纬度取 -60°、-30°、0°、+30°、+60°，另加一个顶视图和一个底视图；所有相机均朝向模型质心。"),
        this);
    gridSummary->setWordWrap(true);
    orbitForm->addRow(gridSummary);

    m_photoButton = new QPushButton(tr("拍摄照片"), this);
    m_photoButton->setObjectName(QStringLiteral("capturePhotoButton"));
    m_photoButton->setProperty("primary", true);

    m_orbitButton = new QPushButton(tr("拍摄 62 个视角"), this);
    m_orbitButton->setObjectName(QStringLiteral("captureOrbitButton"));

    // The render panel resizes the viewport image to the configured output size
    // without preserving the window's aspect ratio, so keep the user informed.
    m_aspectHint = new QLabel(
        tr("图像会缩放到「渲染」页的输出尺寸。"), this);
    m_aspectHint->setWordWrap(true);
    m_aspectHint->setEnabled(false);

    auto* buttonRow = new QHBoxLayout();
    buttonRow->addWidget(m_photoButton);
    buttonRow->addWidget(m_orbitButton);

    m_shots = new QTreeWidget(this);
    m_shots->setObjectName(QStringLiteral("captureShotList"));
    m_shots->setColumnCount(5);
    m_shots->setHeaderLabels({tr("#"), tr("分组"), tr("图像"), tr("方位角"),
                              tr("俯仰角")});
    m_shots->setRootIsDecorated(false);
    m_shots->setUniformRowHeights(true);
    m_shots->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_shots->setSelectionMode(QAbstractItemView::SingleSelection);
    m_shots->setMinimumHeight(120);
    m_shots->header()->setSectionResizeMode(1, QHeaderView::Stretch);

    auto* shotsGroup = new QGroupBox(tr("已拍摄"), this);
    auto* shotsLayout = new QVBoxLayout(shotsGroup);
    shotsLayout->setContentsMargins(6, 6, 6, 6);
    shotsLayout->addWidget(m_shots);

    m_status = new QLabel(tr("就绪"), this);
    m_status->setObjectName(QStringLiteral("captureStatus"));
    m_status->setWordWrap(true);

    auto* contents = new QWidget(this);
    auto* layout = new QVBoxLayout(contents);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);
    layout->addWidget(outputGroup);
    layout->addWidget(orbitGroup);
    layout->addLayout(buttonRow);
    layout->addWidget(m_aspectHint);
    layout->addWidget(m_status);
    m_structuredGroup=new QGroupBox(tr("结构光扫描"),this);
    m_structuredGroup->setObjectName("structuredLightGroup");
    auto* scanForm=new QFormLayout(m_structuredGroup);
    m_renderer=new QComboBox(m_structuredGroup);m_renderer->setObjectName("scanRenderer");
    m_renderer->addItem(tr("VTK · 直接光照"),"vtk");m_renderer->addItem(tr("Mitsuba · 路径追踪"),"mitsuba");
    scanForm->addRow(tr("采集渲染器"),m_renderer);
    m_samples=new QSpinBox(m_structuredGroup);m_samples->setObjectName("scanSamples");
    m_samples->setRange(1,4096);m_samples->setValue(32);m_samples->setEnabled(false);
    m_depth=new QSpinBox(m_structuredGroup);m_depth->setObjectName("scanPathDepth");
    m_depth->setRange(2,16);m_depth->setValue(4);m_depth->setEnabled(false);
    scanForm->addRow(tr("路径追踪采样数"),m_samples);scanForm->addRow(tr("最大光路深度"),m_depth);
    connect(m_renderer,&QComboBox::currentIndexChanged,this,[this] {
        const bool tracing=m_renderer->currentData().toString()=="mitsuba";
        m_samples->setEnabled(tracing);m_depth->setEnabled(tracing);
    });
    const QStringList labels{tr("相机宽度"),tr("相机高度"),tr("投影宽度"),tr("投影高度"),tr("扫描站数")};
    const QStringList names{"scanCameraWidth","scanCameraHeight","scanProjectorWidth","scanProjectorHeight","scanStationCount"};
    const int defaults[]{640,480,256,256,12};
    for(int i=0;i<5;++i) {
        auto* spin=new QSpinBox(m_structuredGroup);m_scanSizes[i]=spin;spin->setObjectName(names[i]);
        spin->setRange(i==4?1:32,i==4?64:4096);spin->setValue(defaults[i]);scanForm->addRow(labels[i],spin);
    }
    m_baseline=new QDoubleSpinBox(m_structuredGroup);m_baseline->setObjectName("scanBaseline");
    m_baseline->setRange(0,10000);m_baseline->setDecimals(3);m_baseline->setSpecialValueText(tr("自动"));
    scanForm->addRow(tr("相机与投影器间距（mm）"),m_baseline);
    m_lightIntensity=new QDoubleSpinBox(m_structuredGroup);m_lightIntensity->setObjectName("scanLightIntensity");
    m_lightIntensity->setRange(.05,100);m_lightIntensity->setDecimals(2);m_lightIntensity->setSingleStep(.5);
    m_lightIntensity->setValue(2.0);
    m_lightIntensity->setToolTip(tr("参考距离处的投影光强。提高会整体变亮，过高会在高光处饱和；需要保留高动态范围时改用多曝光。"));
    scanForm->addRow(tr("投影光强"),m_lightIntensity);
    // Metallic and roughness live on the model material page: the same values
    // light the viewport preview and this capture. Only the decorative maps are
    // captured here, because they describe the part itself.
    const QStringList mapLabels{tr("材质基色贴图"),tr("金属度贴图"),tr("粗糙度贴图")};
    for(int i=0;i<3;++i) {
        auto* row=new QWidget(m_structuredGroup);auto* rowLayout=new QHBoxLayout(row);rowLayout->setContentsMargins(0,0,0,0);
        m_materialMaps[i]=new QLineEdit(row);m_materialMaps[i]->setObjectName(QString("scanMaterialMap%1").arg(i));
        m_materialMaps[i]->setPlaceholderText(tr("可选，描述材料自身"));
        auto* browse=new QPushButton(tr("选择…"),row);
        rowLayout->addWidget(m_materialMaps[i],1);rowLayout->addWidget(browse);scanForm->addRow(mapLabels[i],row);
        connect(browse,&QPushButton::clicked,this,[this,i] {
            const QString path=QFileDialog::getOpenFileName(this,tr("选择材质贴图"),{},tr("图像 (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
            if(!path.isEmpty())m_materialMaps[i]->setText(path);
        });
    }
    m_phase=new QCheckBox(tr("相移条纹精化"),m_structuredGroup);m_phase->setObjectName("scanPhaseShift");
    m_hdr=new QCheckBox(tr("多曝光采集"),m_structuredGroup);m_hdr->setObjectName("scanMultiExposure");
    scanForm->addRow(m_phase);scanForm->addRow(m_hdr);
    auto* scanSingle=new QPushButton(tr("当前视角结构光扫描"),m_structuredGroup);scanSingle->setObjectName("scanSingleButton");
    auto* scanMultiple=new QPushButton(tr("多站结构光扫描"),m_structuredGroup);scanMultiple->setObjectName("scanMultipleButton");
    scanForm->addRow(scanSingle);scanForm->addRow(scanMultiple);
    auto* scanHint=new QLabel(tr("金属度与粗糙度在模型属性页设置，视口预览与拍摄共用。材质贴图保持固定，编码由投影光产生。VTK 只计算直接光；Mitsuba 需完整离线运行库，最大光路深度为 2 时只含直接光照。"),m_structuredGroup);
    scanHint->setWordWrap(true);scanForm->addRow(scanHint);
    layout->addWidget(m_structuredGroup);
    m_scanCancel=new QPushButton(tr("取消结构光拍摄"),this);m_scanCancel->setObjectName("scanCancelButton");m_scanCancel->setEnabled(false);
    layout->addWidget(m_scanCancel);
    layout->addWidget(shotsGroup, 1);
    connect(scanSingle,&QPushButton::clicked,this,[this]{emit structuredScanRequested(false);});
    connect(scanMultiple,&QPushButton::clicked,this,[this]{emit structuredScanRequested(true);});
    connect(m_scanCancel,&QPushButton::clicked,this,&CapturePanel::structuredCancelRequested);

    connect(m_browseButton, &QPushButton::clicked, this, &CapturePanel::browseForDirectory);
    connect(m_photoButton, &QPushButton::clicked, this, &CapturePanel::capturePhotoRequested);
    connect(m_orbitButton, &QPushButton::clicked, this, [this] {
        emit captureOrbitRequested(m_distance->value());
    });
    auto* scroll=new QScrollArea(this);scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);scroll->setWidget(contents);
    auto* outer=new QVBoxLayout(this);outer->setContentsMargins(0,0,0,0);outer->addWidget(scroll);
}

QString CapturePanel::baseDirectory() const
{
    return m_baseDirectory;
}

void CapturePanel::setBaseDirectory(const QString& directory)
{
    if (directory.isEmpty() || directory == m_baseDirectory) {
        return;
    }
    m_baseDirectory = directory;
    updateDirectoryLabel();
}

void CapturePanel::setDefaultDistance(double distance)
{
    m_distance->setValue(distance > 0.0 ? distance : OrbitParameters{}.distance);
}

void CapturePanel::setShots(const std::vector<CaptureShot>& shots)
{
    m_shots->clear();
    for (const CaptureShot& shot : shots) {
        auto* item = new QTreeWidgetItem(m_shots);
        item->setText(0, QString::number(shot.index));
        item->setText(1, shot.groupName);
        item->setText(2, shot.imageName);
        item->setText(3, QString::number(shot.azimuthDeg, 'f', 1));
        item->setText(4, QString::number(shot.elevationDeg, 'f', 1));
    }
    m_shots->resizeColumnToContents(0);
    m_shots->resizeColumnToContents(2);
    m_shots->resizeColumnToContents(3);
    m_shots->resizeColumnToContents(4);
}

void CapturePanel::setStatusText(const QString& text)
{
    m_status->setText(text);
}

void CapturePanel::setBusy(bool busy)
{
    m_photoButton->setEnabled(!busy);
    m_orbitButton->setEnabled(!busy);
    m_browseButton->setEnabled(!busy);
    m_distance->setEnabled(!busy);
    m_structuredGroup->setEnabled(!busy);
}

void CapturePanel::setStructuredBusy(bool busy) {setBusy(busy);m_scanCancel->setEnabled(busy);}

capture::ScanOptions CapturePanel::structuredOptions() const {
    capture::ScanOptions result;
    result.renderBackend=m_renderer->currentData().toString();result.samplesPerPixel=m_samples->value();
    result.maxPathDepth=m_depth->value();
    result.cameraWidth=m_scanSizes[0]->value();result.cameraHeight=m_scanSizes[1]->value();
    result.projectorWidth=m_scanSizes[2]->value();result.projectorHeight=m_scanSizes[3]->value();
    result.stationCount=m_scanSizes[4]->value();result.distance=m_distance->value();
    result.baseline=m_baseline->value();result.lightIntensity=m_lightIntensity->value();
    result.phaseShift=m_phase->isChecked();
    result.multiExposure=m_hdr->isChecked();
    result.material.baseColorMap=m_materialMaps[0]->text().trimmed();
    result.material.metallicMap=m_materialMaps[1]->text().trimmed();
    result.material.roughnessMap=m_materialMaps[2]->text().trimmed();
    return result;
}

void CapturePanel::setStructuredOptions(const capture::ScanOptions& options) {
    m_renderer->setCurrentIndex(m_renderer->findData(options.renderBackend));
    m_samples->setValue(options.samplesPerPixel);m_depth->setValue(options.maxPathDepth);
    const int sizes[]{options.cameraWidth,options.cameraHeight,options.projectorWidth,options.projectorHeight,options.stationCount};
    for(int i=0;i<5;++i)m_scanSizes[i]->setValue(sizes[i]);
    m_baseline->setValue(options.baseline);m_lightIntensity->setValue(options.lightIntensity);
    m_phase->setChecked(options.phaseShift);
    m_hdr->setChecked(options.multiExposure);
    m_materialMaps[0]->setText(options.material.baseColorMap);m_materialMaps[1]->setText(options.material.metallicMap);
    m_materialMaps[2]->setText(options.material.roughnessMap);
    if(options.distance>0)m_distance->setValue(options.distance);
}

void CapturePanel::browseForDirectory()
{
    const QString directory = QFileDialog::getExistingDirectory(
        this, tr("拍摄输出目录"), m_baseDirectory);
    if (directory.isEmpty()) {
        return;
    }
    m_baseDirectory = directory;
    updateDirectoryLabel();
    emit baseDirectoryChanged(directory);
}

void CapturePanel::updateDirectoryLabel()
{
    m_directoryLabel->setText(m_baseDirectory);
    m_directoryLabel->setToolTip(m_baseDirectory);
}
