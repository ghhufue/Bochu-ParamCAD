#include "ui/panels/ReconstructPanel.h"

#include "app/ProjectPaths.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleValidator>
#include <QFileInfo>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QVBoxLayout>

ReconstructPanel::ReconstructPanel(QWidget* parent)
    : QWidget(parent)
{
    m_captureBaseDirectory = ProjectPaths::capturesRoot();

    // Editable rather than read-only so a path can be pasted in, which is also
    // what the UI automation relies on to fill the field.
    m_imageDirectory = new QLineEdit(this);
    m_imageDirectory->setObjectName(QStringLiteral("reconstructImageDir"));
    m_imageDirectory->setToolTip(tr("用于重建的多视角图像文件夹"));

    m_browseImagesButton = new QPushButton(tr("浏览…"), this);
    m_browseImagesButton->setObjectName(QStringLiteral("reconstructBrowseButton"));

    m_useLatestButton = new QPushButton(tr("使用最近一次拍摄"), this);
    m_useLatestButton->setObjectName(QStringLiteral("reconstructUseLatestButton"));

    auto* imagesRow = new QWidget(this);
    auto* imagesLayout = new QHBoxLayout(imagesRow);
    imagesLayout->setContentsMargins(0, 0, 0, 0);
    imagesLayout->setSpacing(4);
    imagesLayout->addWidget(m_imageDirectory, 1);
    imagesLayout->addWidget(m_browseImagesButton);

    auto* imagesGroup = new QGroupBox(tr("图像"), this);
    auto* imagesForm = new QFormLayout(imagesGroup);
    imagesForm->addRow(tr("目录"), imagesRow);
    imagesForm->addRow(QString(), m_useLatestButton);

    m_outputDirectory = new QLineEdit(this);
    m_outputDirectory->setObjectName(QStringLiteral("reconstructOutputDir"));
    m_outputDirectory->setText(QDir::cleanPath(ProjectPaths::reconstructionsRoot()));
    m_outputDirectory->setToolTip(tr("每次运行都会在此处创建独立的时间戳子目录"));

    m_browseOutputButton = new QPushButton(tr("浏览…"), this);
    m_browseOutputButton->setObjectName(QStringLiteral("reconstructOutputBrowseButton"));

    auto* outputRow = new QWidget(this);
    auto* outputLayout = new QHBoxLayout(outputRow);
    outputLayout->setContentsMargins(0, 0, 0, 0);
    outputLayout->setSpacing(4);
    outputLayout->addWidget(m_outputDirectory, 1);
    outputLayout->addWidget(m_browseOutputButton);

    auto* outputGroup = new QGroupBox(tr("输出"), this);
    auto* outputForm = new QFormLayout(outputGroup);
    outputForm->addRow(tr("目录"), outputRow);

    m_useCameraInfo = new QCheckBox(tr("使用拍摄位姿与内参"), this);
    m_useCameraInfo->setObjectName(QStringLiteral("reconstructUseCameraInfo"));
    m_useCameraInfo->setChecked(true);

    m_dense = new QCheckBox(tr("稠密重建（GPU）"), this);
    m_dense->setObjectName(QStringLiteral("reconstructDense"));

    m_matcher = new QComboBox(this);
    m_matcher->setObjectName(QStringLiteral("reconstructMatcher"));
    m_matcher->addItems({tr("穷举匹配"), tr("顺序匹配")});

    auto* optionsGroup = new QGroupBox(tr("选项"), this);
    auto* optionsForm = new QFormLayout(optionsGroup);
    m_engine=new QComboBox(this);m_engine->setObjectName("reconstructEngine");
    m_engine->addItem(tr("按采集类型自动选择"),"auto");m_engine->addItem(tr("COLMAP 普通多视角图片"),"colmap");
    m_engine->addItem(tr("编码结构光"),"structured_light");optionsForm->addRow(tr("重建方式"),m_engine);
    optionsForm->addRow(QString(), m_useCameraInfo);
    optionsForm->addRow(QString(), m_dense);
    optionsForm->addRow(tr("匹配器"), m_matcher);
    m_minContrast=new QDoubleSpinBox(this);m_minContrast->setObjectName("reconstructMinContrast");
    m_minContrast->setRange(.1,253);m_minContrast->setValue(8);
    optionsForm->addRow(tr("结构光最小明暗差"),m_minContrast);

    m_runButton = new QPushButton(tr("重建"), this);
    m_runButton->setObjectName(QStringLiteral("reconstructRunButton"));
    m_runButton->setProperty("primary", true);

    m_cancelButton = new QPushButton(tr("取消"), this);
    m_cancelButton->setObjectName(QStringLiteral("reconstructCancelButton"));
    m_cancelButton->setEnabled(false);

    auto* buttonRow = new QHBoxLayout();
    buttonRow->addWidget(m_runButton);
    buttonRow->addWidget(m_cancelButton);

    m_progress = new QProgressBar(this);
    m_progress->setObjectName(QStringLiteral("reconstructProgress"));
    m_progress->setRange(0, 4);
    m_progress->setValue(0);
    m_progress->setFormat(tr("%1/%2").arg(0).arg(4));

    m_status = new QLabel(tr("就绪"), this);
    m_status->setObjectName(QStringLiteral("reconstructStatus"));
    m_status->setWordWrap(true);

    auto* downsample = new QGroupBox(tr("降采样（体素融合）"), this);
    downsample->setObjectName("downsampleGroup");
    auto* downsampleForm = new QFormLayout(downsample);
    m_downsampleSpacing = new QComboBox(downsample);
    m_downsampleSpacing->setObjectName("downsampleSpacing");
    m_downsampleSpacing->setEditable(true);
    m_downsampleSpacing->setInsertPolicy(QComboBox::NoInsert);
    auto* spacingValidator = new QDoubleValidator(0.001, 1000.0, 3, m_downsampleSpacing);
    spacingValidator->setNotation(QDoubleValidator::StandardNotation);
    spacingValidator->setLocale(QLocale::c());
    m_downsampleSpacing->setValidator(spacingValidator);
    m_downsampleSpacing->addItem(tr("原始点云（不降采样）"), 0.0);
    m_downsampleSpacing->addItem(QStringLiteral("10"), 10.0);
    m_downsampleSpacing->setCurrentIndex(1);
    downsampleForm->addRow(tr("融合间距（mm）"), m_downsampleSpacing);
    m_downsampleRun = new QPushButton(tr("运行降采样"), downsample);
    m_downsampleRun->setObjectName("downsampleRunButton");
    m_downsampleRun->setEnabled(false);
    downsampleForm->addRow(m_downsampleRun);
    m_downsampleStatus = new QLabel(
        tr("结果按间距缓存在原文件旁的 downsampled 目录；已有尺度可直接选择载入。"), downsample);
    m_downsampleStatus->setObjectName("downsampleStatus");
    m_downsampleStatus->setWordWrap(true);
    downsampleForm->addRow(m_downsampleStatus);
    m_downsampleGroup = downsample;

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);
    layout->addWidget(imagesGroup);
    layout->addWidget(outputGroup);
    layout->addWidget(optionsGroup);
    layout->addLayout(buttonRow);
    layout->addWidget(m_progress);
    layout->addWidget(m_status);
    layout->addWidget(downsample);
    layout->addStretch(1);

    const QString latest = latestCaptureDirectory();
    if (!latest.isEmpty()) {
        setImageDirectory(latest);
    }

    connect(m_browseImagesButton, &QPushButton::clicked, this, &ReconstructPanel::browseForImages);
    connect(m_browseOutputButton, &QPushButton::clicked, this, &ReconstructPanel::browseForOutput);
    connect(m_useLatestButton, &QPushButton::clicked, this, &ReconstructPanel::useLatestCapture);
    connect(m_runButton, &QPushButton::clicked, this, &ReconstructPanel::startReconstruction);
    connect(m_cancelButton, &QPushButton::clicked, this, &ReconstructPanel::cancelRequested);
    connect(m_downsampleRun, &QPushButton::clicked, this, [this] {
        emit downsampleRequested(downsampleSpacing());
    });
    connect(m_downsampleSpacing, &QComboBox::activated, this, [this](int index) {
        bool valid = false;
        const double value = m_downsampleSpacing->itemData(index).toDouble(&valid);
        if (valid) emit downsampleRequested(value);
    });
    connect(m_engine,&QComboBox::currentIndexChanged,this,&ReconstructPanel::updateBackendControls);
    connect(m_imageDirectory,&QLineEdit::textChanged,this,&ReconstructPanel::updateBackendControls);
    updateBackendControls();
}

QString ReconstructPanel::imageDirectory() const
{
    return m_imageDirectory->text().trimmed();
}

void ReconstructPanel::setImageDirectory(const QString& directory)
{
    m_imageDirectory->setText(QDir::cleanPath(directory));
    m_imageDirectory->setToolTip(m_imageDirectory->text());
}

QString ReconstructPanel::outputRoot() const
{
    return m_outputDirectory->text().trimmed();
}

void ReconstructPanel::setOutputRoot(const QString& directory)
{
    m_outputDirectory->setText(QDir::cleanPath(directory));
    m_outputDirectory->setToolTip(m_outputDirectory->text());
}

void ReconstructPanel::setRequest(const ReconstructRequest& request)
{
    setImageDirectory(request.imageDirectory);setOutputRoot(request.outputRoot);
    m_engine->setCurrentIndex(m_engine->findData(request.engine));
    m_useCameraInfo->setChecked(request.useCameraInfo);m_dense->setChecked(request.dense);
    m_matcher->setCurrentIndex(request.matcher=="sequential"?1:0);
    m_minContrast->setValue(request.minContrast);
    const QSignalBlocker blocker(m_downsampleSpacing);
    m_downsampleSpacing->setCurrentText(QString::number(request.voxelSizeMm,'g',12));
    updateBackendControls();
}

void ReconstructPanel::setCaptureBaseDirectory(const QString& directory)
{
    if (directory.isEmpty() || directory == m_captureBaseDirectory) {
        return;
    }
    m_captureBaseDirectory = directory;
    if (m_imageDirectory->text().isEmpty()) {
        const QString latest = latestCaptureDirectory();
        if (!latest.isEmpty()) {
            setImageDirectory(latest);
        }
    }
}

void ReconstructPanel::setStatusText(const QString& text)
{
    m_status->setText(text);
}

void ReconstructPanel::setBusy(bool busy)
{
    m_busy=busy;m_engine->setEnabled(!busy);
    m_runButton->setEnabled(!busy);
    m_cancelButton->setEnabled(busy);
    m_imageDirectory->setEnabled(!busy);
    m_outputDirectory->setEnabled(!busy);
    m_browseImagesButton->setEnabled(!busy);
    m_browseOutputButton->setEnabled(!busy);
    m_useLatestButton->setEnabled(!busy);
    m_useCameraInfo->setEnabled(!busy);
    m_dense->setEnabled(!busy);
    m_matcher->setEnabled(!busy);
    updateBackendControls();
}

void ReconstructPanel::updateBackendControls() {
    const QString engine=m_engine->currentData().toString();
    const bool structured=engine=="structured_light" || (engine=="auto"
        && QFileInfo::exists(QDir(imageDirectory()).filePath("capture.json")));
    m_useCameraInfo->setEnabled(!m_busy && !structured);m_dense->setEnabled(!m_busy && !structured);
    m_matcher->setEnabled(!m_busy && !structured);
    m_minContrast->setEnabled(!m_busy && structured);
}

void ReconstructPanel::setProgress(int value, int maximum)
{
    if (maximum > 0) {
        m_progress->setRange(0, maximum);
    }
    m_progress->setValue(value);
    m_progress->setFormat(tr("%1/%2").arg(value).arg(m_progress->maximum()));
}

QString ReconstructPanel::latestCaptureDirectory() const
{
    // Capture groups are named yyyyMMdd_HHmmss (with a -N suffix after a
    // collision). The base directory defaults to outputs/captures, which holds
    // nothing else, but it stays user-selectable and may point at a folder with
    // unrelated entries, so only matching names are considered.
    static const QRegularExpression stamp(QStringLiteral("^\\d{8}_\\d{6}(-\\d+)?$"));
    const QDir base(m_captureBaseDirectory);
    const QStringList entries =
        base.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    QString latest;
    for (const QString& entry : entries) {
        if (stamp.match(entry).hasMatch()) {
            // A plain timestamp sorts chronologically as text, and the -N suffix
            // sorts after its base name, so the last match is the newest group.
            latest = entry;
        }
    }
    return latest.isEmpty() ? QString() : base.filePath(latest);
}

void ReconstructPanel::browseForImages()
{
    const QString directory =
        QFileDialog::getExistingDirectory(this, tr("图像目录"), imageDirectory());
    if (directory.isEmpty()) {
        return;
    }
    setImageDirectory(directory);
}

void ReconstructPanel::browseForOutput()
{
    const QString directory =
        QFileDialog::getExistingDirectory(this, tr("输出目录"), outputRoot());
    if (directory.isEmpty()) {
        return;
    }
    setOutputRoot(directory);
}

void ReconstructPanel::useLatestCapture()
{
    const QString latest = latestCaptureDirectory();
    if (latest.isEmpty()) {
        setStatusText(tr("在 %1 中未找到拍摄文件夹").arg(m_captureBaseDirectory));
        return;
    }
    setImageDirectory(latest);
    setStatusText(tr("使用 %1").arg(latest));
}

void ReconstructPanel::startReconstruction()
{
    ReconstructRequest request;
    request.imageDirectory = imageDirectory();
    request.outputRoot = outputRoot();
    request.engine=m_engine->currentData().toString();
    request.voxelSizeMm=0.2;request.minContrast=m_minContrast->value();
    request.useCameraInfo = m_useCameraInfo->isChecked();
    request.dense = m_dense->isChecked();
    request.matcher = m_matcher->currentIndex() == 1 ? QStringLiteral("sequential")
                                                     : QStringLiteral("exhaustive");
    emit reconstructRequested(request);
}

double ReconstructPanel::downsampleSpacing() const
{
    bool valid = false;
    const double value = m_downsampleSpacing->currentText().trimmed().toDouble(&valid);
    return valid && value > 0.0 ? value : 0.0;
}

void ReconstructPanel::setDownsampleScales(const QVector<double>& scales, double currentSpacing)
{
    const QSignalBlocker blocker(m_downsampleSpacing);
    bool typedValid = false;
    const double typedValue = m_downsampleSpacing->currentText().trimmed().toDouble(&typedValid);
    m_downsampleSpacing->clear();
    m_downsampleSpacing->addItem(tr("原始点云（不降采样）"), 0.0);
    for (double scale : scales)
        m_downsampleSpacing->addItem(QString::number(scale, 'g', 12), scale);
    if (currentSpacing > 0.0) {
        int index = m_downsampleSpacing->findData(currentSpacing);
        if (index < 0) {
            m_downsampleSpacing->addItem(QString::number(currentSpacing, 'g', 12), currentSpacing);
            index = m_downsampleSpacing->count() - 1;
        }
        m_downsampleSpacing->setCurrentIndex(index);
    } else if (typedValid && typedValue > 0.0) {
        // Keep the spacing the user last worked with instead of snapping back
        // to "original" whenever a plain cloud is opened.
        m_downsampleSpacing->setCurrentText(QString::number(typedValue, 'g', 12));
        m_downsampleSpacing->setCurrentIndex(-1);
    } else {
        m_downsampleSpacing->setCurrentIndex(0);
    }
}

void ReconstructPanel::setDownsampleStatus(const QString& text)
{
    m_downsampleStatus->setText(text);
}

void ReconstructPanel::setDownsampleAvailable(bool available)
{
    m_downsampleAvailable = available;
    m_downsampleRun->setEnabled(available && !m_downsampleBusy);
    if (!available) m_downsampleStatus->setText(tr("载入点云后可按间距生成降采样点云。"));
}

void ReconstructPanel::setDownsampleBusy(bool busy)
{
    m_downsampleBusy = busy;
    m_downsampleSpacing->setEnabled(!busy);
    m_downsampleRun->setEnabled(m_downsampleAvailable && !busy);
}
