using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Forms;

internal sealed class MotorSample
{
    public DateTime Time;
    public ushort Sequence;
    public byte Mode;
    public float RightTarget;
    public float RightActual;
    public float RightPwm;
    public float LeftTarget;
    public float LeftActual;
    public float LeftPwm;
}

internal sealed class SpeedPlot : Control
{
    private readonly List<float> right = new List<float>();
    private readonly List<float> left = new List<float>();
    private const int Capacity = 240;

    public SpeedPlot()
    {
        DoubleBuffered = true;
        BackColor = Color.FromArgb(16, 24, 32);
        MinimumSize = new Size(200, 130);
    }

    public void Add(float rightRpm, float leftRpm)
    {
        right.Add(rightRpm);
        left.Add(leftRpm);
        if (right.Count > Capacity) right.RemoveAt(0);
        if (left.Count > Capacity) left.RemoveAt(0);
        Invalidate();
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        Graphics g = e.Graphics;
        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
        int pad = 16;
        int mid = Height / 2;
        using (Pen axis = new Pen(Color.FromArgb(75, 91, 104)))
            g.DrawLine(axis, pad, mid, Width - pad, mid);
        using (Brush label = new SolidBrush(Color.FromArgb(132, 151, 166)))
        {
            g.DrawString("+RPM", Font, label, pad, 3);
            g.DrawString("-RPM", Font, label, pad, Height - 18);
        }
        if (right.Count < 2)
        {
            DrawHint(g, "等待小车运行数据…");
            return;
        }
        float peak = 30.0f;
        float observedPeak = 0.0f;
        foreach (float value in right) { observedPeak = Math.Max(observedPeak, Math.Abs(value)); peak = Math.Max(peak, Math.Abs(value)); }
        foreach (float value in left) { observedPeak = Math.Max(observedPeak, Math.Abs(value)); peak = Math.Max(peak, Math.Abs(value)); }
        peak *= 1.15f;
        DrawSeries(g, right, peak, Color.FromArgb(66, 165, 245));
        DrawSeries(g, left, peak, Color.FromArgb(102, 217, 139));
        if (observedPeak < 0.5f) DrawHint(g, "当前停车，等待车轮转动");
    }

    private void DrawHint(Graphics g, string text)
    {
        using (Brush brush = new SolidBrush(Color.FromArgb(174, 188, 199)))
        using (StringFormat format = new StringFormat { Alignment = StringAlignment.Center, LineAlignment = StringAlignment.Center })
            g.DrawString(text, Font, brush, ClientRectangle, format);
    }

    private void DrawSeries(Graphics g, List<float> values, float peak, Color color)
    {
        int pad = 16;
        float usableWidth = Math.Max(1, Width - pad * 2);
        float usableHalfHeight = Math.Max(1, Height / 2.0f - pad);
        PointF[] points = new PointF[values.Count];
        for (int i = 0; i < values.Count; i++)
        {
            float x = pad + i * usableWidth / Math.Max(1, values.Count - 1);
            float y = Height / 2.0f - values[i] / peak * usableHalfHeight;
            points[i] = new PointF(x, y);
        }
        using (Pen pen = new Pen(color, 2.0f)) g.DrawLines(pen, points);
    }
}

internal sealed class CarRemoteForm : Form
{
    private const int BaudRate = 921600;
    private const double WheelDiameterM = 0.080;
    private const double MaxRpm = 360.0;
    private const double MaxDistanceM = 20.0;
    private const double CurveInnerWheelScale = 0.45;

    private readonly ComboBox portBox = new ComboBox();
    private readonly Button connectButton = new Button();
    private readonly TextBox commonRpmBox = new TextBox();
    private readonly TextBox rightRpmBox = new TextBox();
    private readonly TextBox leftRpmBox = new TextBox();
    private readonly TextBox distanceBox = new TextBox();
    private readonly Label connectionLabel = new Label();
    private readonly Label modeLabel = new Label();
    private readonly Label rightLabel = new Label();
    private readonly Label leftLabel = new Label();
    private readonly Label distanceLabel = new Label();
    private readonly ProgressBar progress = new ProgressBar();
    private readonly SpeedPlot plot = new SpeedPlot();
    private readonly Timer timer = new Timer();
    private readonly ConcurrentQueue<MotorSample> samples = new ConcurrentQueue<MotorSample>();
    private readonly List<byte> receiveBuffer = new List<byte>();
    private readonly object receiveLock = new object();
    private readonly object sendLock = new object();

    private SerialPort serial;
    private DateTime lastKeepAlive = DateTime.MinValue;
    private DateTime? lastDistanceSample;
    private bool distanceRunning;
    private double distanceTarget;
    private double distanceDone;
    private int distanceDirection;
    private int heldDirection;
    private bool keyForward;
    private bool keyBackward;
    private bool keyLeft;
    private bool keyRight;

    public CarRemoteForm()
    {
        Text = "CCD 小车 · 蓝牙遥控与里程测试（弧线转向版）";
        ClientSize = new Size(940, 820);
        MinimumSize = new Size(820, 700);
        Font = new Font("Microsoft YaHei UI", 9.0f);
        KeyPreview = true;
        KeyDown += delegate(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Space)
            {
                ClearDriveKeys();
                EmergencyStop();
                e.Handled = true;
                e.SuppressKeyPress = true;
                return;
            }
            if (SetDriveKey(e.KeyCode, true))
            {
                ApplyKeyboardDrive();
                e.Handled = true;
                e.SuppressKeyPress = true;
            }
        };
        KeyUp += delegate(object sender, KeyEventArgs e)
        {
            if (SetDriveKey(e.KeyCode, false))
            {
                ApplyKeyboardDrive();
                e.Handled = true;
                e.SuppressKeyPress = true;
            }
        };
        FormClosing += delegate { Disconnect(); };

        BuildUi();
        RefreshPorts();
        timer.Interval = 40;
        timer.Tick += TimerTick;
        timer.Start();
    }

    private static Label Caption(string text)
    {
        return new Label { Text = text, AutoSize = true, Margin = new Padding(3, 7, 3, 0) };
    }

    private void BuildUi()
    {
        TableLayoutPanel root = new TableLayoutPanel();
        root.Dock = DockStyle.Fill;
        root.Padding = new Padding(14);
        root.ColumnCount = 1;
        root.RowCount = 8;
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        Controls.Add(root);

        TableLayoutPanel header = new TableLayoutPanel { Dock = DockStyle.Fill, AutoSize = true, ColumnCount = 2 };
        header.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 55));
        header.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 45));
        Label title = new Label { Text = "CCD 小车遥控台", AutoSize = true, Font = new Font(Font.FontFamily, 17, FontStyle.Bold) };
        connectionLabel.Text = "未连接";
        connectionLabel.AutoSize = true;
        connectionLabel.Anchor = AnchorStyles.Right;
        header.Controls.Add(title, 0, 0);
        header.Controls.Add(connectionLabel, 1, 0);
        root.Controls.Add(header);

        FlowLayoutPanel connection = Section("蓝牙串口");
        portBox.DropDownStyle = ComboBoxStyle.DropDownList;
        portBox.Width = 180;
        connection.Controls.Add(portBox);
        connection.Controls.Add(MakeButton("刷新", delegate { RefreshPorts(); }));
        connectButton.Text = "连接";
        connectButton.AutoSize = true;
        connectButton.Click += delegate { if (IsConnected) Disconnect(); else Connect(); };
        connection.Controls.Add(connectButton);
        connection.Controls.Add(Caption("HC-05：" + BaudRate + " 8N1"));
        root.Controls.Add(connection);

        FlowLayoutPanel speeds = Section("速度设置");
        commonRpmBox.Text = "60";
        rightRpmBox.Text = "60";
        leftRpmBox.Text = "60";
        commonRpmBox.Width = rightRpmBox.Width = leftRpmBox.Width = 62;
        speeds.Controls.Add(Caption("统一 RPM"));
        speeds.Controls.Add(commonRpmBox);
        speeds.Controls.Add(MakeButton("统一到左右轮", CopyRpm));
        speeds.Controls.Add(Caption("右轮 RPM"));
        speeds.Controls.Add(rightRpmBox);
        speeds.Controls.Add(Caption("左轮 RPM"));
        speeds.Controls.Add(leftRpmBox);
        speeds.Controls.Add(MakeButton("连续前进", delegate { RunContinuous(1); }));
        speeds.Controls.Add(MakeButton("连续后退", delegate { RunContinuous(-1); }));
        root.Controls.Add(speeds);

        GroupBox remoteBox = new GroupBox();
        remoteBox.Text = "遥控车模式（斜向按钮可边走边转；按住行驶，松开停车）";
        remoteBox.Dock = DockStyle.Fill;
        remoteBox.AutoSize = true;
        remoteBox.Padding = new Padding(8);
        TableLayoutPanel remotePad = new TableLayoutPanel();
        remotePad.AutoSize = true;
        remotePad.Anchor = AnchorStyles.None;
        remotePad.ColumnCount = 3;
        remotePad.RowCount = 3;
        remotePad.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 116));
        remotePad.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 116));
        remotePad.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 116));
        remotePad.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
        remotePad.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
        remotePad.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
        remotePad.Controls.Add(MakeDriveButton("↖ 前进左转", 4), 0, 0);
        remotePad.Controls.Add(MakeDriveButton("▲ 前进", 1), 1, 0);
        remotePad.Controls.Add(MakeDriveButton("前进右转 ↗", 5), 2, 0);
        remotePad.Controls.Add(MakeDriveButton("◀ 原地左转", 2), 0, 1);
        Button padStop = MakeButton("■ 停止", EmergencyStop);
        padStop.Dock = DockStyle.Fill;
        remotePad.Controls.Add(padStop, 1, 1);
        remotePad.Controls.Add(MakeDriveButton("原地右转 ▶", 3), 2, 1);
        remotePad.Controls.Add(MakeDriveButton("↙ 后退左转", -2), 0, 2);
        remotePad.Controls.Add(MakeDriveButton("▼ 后退", -1), 1, 2);
        remotePad.Controls.Add(MakeDriveButton("后退右转 ↘", -3), 2, 2);
        remoteBox.Controls.Add(remotePad);
        root.Controls.Add(remoteBox);

        FlowLayoutPanel distance = Section("按距离行驶");
        distanceBox.Text = "1.0";
        distanceBox.Width = 65;
        distance.Controls.Add(Caption("距离（米）"));
        distance.Controls.Add(distanceBox);
        distance.Controls.Add(MakeButton("前进指定距离", delegate { StartDistance(1); }));
        distance.Controls.Add(MakeButton("后退指定距离", delegate { StartDistance(-1); }));
        distance.Controls.Add(MakeButton("前进 1 m", delegate { distanceBox.Text = "1"; StartDistance(1); }));
        distance.Controls.Add(MakeButton("后退 1 m", delegate { distanceBox.Text = "1"; StartDistance(-1); }));
        root.Controls.Add(distance);

        Button stop = new Button();
        stop.Text = "■ 急停（空格键）";
        stop.Dock = DockStyle.Fill;
        stop.Height = 42;
        stop.Font = new Font(Font.FontFamily, 12, FontStyle.Bold);
        stop.BackColor = Color.FromArgb(221, 75, 68);
        stop.ForeColor = Color.White;
        stop.FlatStyle = FlatStyle.Flat;
        stop.Click += delegate { EmergencyStop(); };
        root.Controls.Add(stop);

        TableLayoutPanel status = new TableLayoutPanel();
        status.Dock = DockStyle.Fill;
        status.AutoSize = true;
        status.ColumnCount = 2;
        status.RowCount = 4;
        status.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        status.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        modeLabel.Text = "模式：停止";
        modeLabel.Font = new Font("Consolas", 12, FontStyle.Bold);
        modeLabel.AutoSize = true;
        rightLabel.Text = "右轮：等待数据";
        leftLabel.Text = "左轮：等待数据";
        rightLabel.Font = leftLabel.Font = new Font("Consolas", 11, FontStyle.Bold);
        rightLabel.AutoSize = leftLabel.AutoSize = true;
        distanceLabel.Text = "里程任务：未开始";
        distanceLabel.AutoSize = true;
        progress.Dock = DockStyle.Fill;
        status.Controls.Add(modeLabel, 0, 0);
        status.SetColumnSpan(modeLabel, 2);
        status.Controls.Add(rightLabel, 0, 1);
        status.Controls.Add(leftLabel, 1, 1);
        status.Controls.Add(distanceLabel, 0, 2);
        status.SetColumnSpan(distanceLabel, 2);
        status.Controls.Add(progress, 0, 3);
        status.SetColumnSpan(progress, 2);
        root.Controls.Add(status);

        GroupBox graphBox = new GroupBox();
        graphBox.Text = "转速波形（蓝：右轮，绿：左轮）";
        graphBox.Dock = DockStyle.Fill;
        graphBox.Padding = new Padding(8);
        plot.Dock = DockStyle.Fill;
        graphBox.Controls.Add(plot);
        root.Controls.Add(graphBox);
    }

    private static FlowLayoutPanel Section(string name)
    {
        FlowLayoutPanel panel = new FlowLayoutPanel();
        panel.Dock = DockStyle.Fill;
        panel.AutoSize = true;
        panel.WrapContents = true;
        panel.Padding = new Padding(4);
        panel.Margin = new Padding(0, 7, 0, 0);
        panel.BorderStyle = BorderStyle.FixedSingle;
        panel.Controls.Add(new Label { Text = name + "：", AutoSize = true, Font = new Font("Microsoft YaHei UI", 9, FontStyle.Bold), Margin = new Padding(3, 7, 8, 0) });
        return panel;
    }

    private static Button MakeButton(string text, Action action)
    {
        Button button = new Button { Text = text, AutoSize = true };
        button.Click += delegate { action(); };
        return button;
    }

    private Button MakeDriveButton(string text, int direction)
    {
        Button button = new Button();
        button.Text = text;
        button.Dock = DockStyle.Fill;
        button.Font = new Font(Font.FontFamily, 10, FontStyle.Bold);
        button.MouseDown += delegate { BeginManualDirection(direction); };
        button.MouseUp += delegate { EndManualDirection(); };
        return button;
    }

    private bool SetDriveKey(Keys key, bool pressed)
    {
        if (key == Keys.Up || key == Keys.W) keyForward = pressed;
        else if (key == Keys.Down || key == Keys.S) keyBackward = pressed;
        else if (key == Keys.Left || key == Keys.A) keyLeft = pressed;
        else if (key == Keys.Right || key == Keys.D) keyRight = pressed;
        else return false;
        return true;
    }

    private void ClearDriveKeys()
    {
        keyForward = keyBackward = keyLeft = keyRight = false;
    }

    private void ApplyKeyboardDrive()
    {
        int forward = (keyForward ? 1 : 0) - (keyBackward ? 1 : 0);
        int turn = (keyRight ? 1 : 0) - (keyLeft ? 1 : 0);
        int direction = 0;
        if (forward > 0 && turn < 0) direction = 4;
        else if (forward > 0 && turn > 0) direction = 5;
        else if (forward < 0 && turn < 0) direction = -2;
        else if (forward < 0 && turn > 0) direction = -3;
        else if (forward > 0) direction = 1;
        else if (forward < 0) direction = -1;
        else if (turn < 0) direction = 2;
        else if (turn > 0) direction = 3;

        if (direction == 0) EndManualDirection();
        else if (direction != heldDirection) BeginManualDirection(direction);
    }

    private bool IsConnected { get { return serial != null && serial.IsOpen; } }

    private void RefreshPorts()
    {
        string selected = portBox.SelectedItem as string;
        string[] ports = SerialPort.GetPortNames().OrderBy(x => x).ToArray();
        portBox.Items.Clear();
        portBox.Items.AddRange(ports);
        if (ports.Contains("COM17")) portBox.SelectedItem = "COM17";
        else if (selected != null && ports.Contains(selected)) portBox.SelectedItem = selected;
        else if (ports.Length > 0) portBox.SelectedIndex = 0;
    }

    private void Connect()
    {
        if (portBox.SelectedItem == null)
        {
            MessageBox.Show("请选择 HC-05 的蓝牙串口。", "未选择串口", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }
        try
        {
            serial = new SerialPort((string)portBox.SelectedItem, BaudRate, Parity.None, 8, StopBits.One);
            serial.ReadTimeout = 50;
            serial.WriteTimeout = 500;
            serial.DataReceived += SerialDataReceived;
            serial.Open();
            serial.DiscardInBuffer();
            Send("STOP", true);
            Send("STREAM,MOTOR", true);
            connectButton.Text = "断开";
            portBox.Enabled = false;
            connectionLabel.Text = "已连接 " + serial.PortName;
            lastKeepAlive = DateTime.Now;
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "串口连接失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
            Disconnect();
        }
    }

    private void Disconnect()
    {
        distanceRunning = false;
        if (serial != null)
        {
            try { if (serial.IsOpen) Send("STOP", true); } catch { }
            try { serial.DataReceived -= SerialDataReceived; serial.Close(); serial.Dispose(); } catch { }
            serial = null;
        }
        connectButton.Text = "连接";
        portBox.Enabled = true;
        connectionLabel.Text = "未连接；小车已停车";
        modeLabel.Text = "模式：停止";
    }

    private bool Send(string command, bool quiet)
    {
        if (!IsConnected)
        {
            if (!quiet) MessageBox.Show("请先连接蓝牙串口。", "未连接", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return false;
        }
        try
        {
            lock (sendLock) serial.Write(command.Trim() + "\n");
            return true;
        }
        catch (Exception ex)
        {
            if (!quiet) MessageBox.Show(ex.Message, "发送失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
            return false;
        }
    }

    private void CopyRpm()
    {
        double rpm;
        if (!TryNumber(commonRpmBox.Text, "统一 RPM", 0.01, MaxRpm, out rpm)) return;
        rightRpmBox.Text = rpm.ToString("0.###", CultureInfo.InvariantCulture);
        leftRpmBox.Text = rightRpmBox.Text;
    }

    private bool ReadWheelRpm(out double rightRpm, out double leftRpm)
    {
        rightRpm = leftRpm = 0;
        return TryNumber(rightRpmBox.Text, "右轮 RPM", 0.01, MaxRpm, out rightRpm)
            && TryNumber(leftRpmBox.Text, "左轮 RPM", 0.01, MaxRpm, out leftRpm);
    }

    private static bool TryNumber(string text, string name, double minimum, double maximum, out double value)
    {
        bool ok = double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value)
            || double.TryParse(text, out value);
        if (!ok || value < minimum || value > maximum)
        {
            MessageBox.Show(name + " 必须在 " + minimum + "～" + maximum + " 之间。", "参数错误", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return false;
        }
        return true;
    }

    private void RunContinuous(int direction)
    {
        double rightRpm, leftRpm;
        if (!ReadWheelRpm(out rightRpm, out leftRpm)) return;
        distanceRunning = false;
        progress.Value = 0;
        distanceLabel.Text = "里程任务：连续运行，按空格键停车";
        SendDual(direction * rightRpm, direction * leftRpm);
    }

    private void BeginManualDirection(int direction)
    {
        if (direction == heldDirection) return;
        double rightRpm, leftRpm;
        if (!ReadWheelRpm(out rightRpm, out leftRpm)) return;
        distanceRunning = false;
        heldDirection = direction;
        progress.Value = 0;
        if (direction == 1)
        {
            distanceLabel.Text = "遥控：前进（松开即停车）";
            SendDual(rightRpm, leftRpm);
        }
        else if (direction == -1)
        {
            distanceLabel.Text = "遥控：后退（松开即停车）";
            SendDual(-rightRpm, -leftRpm);
        }
        else if (direction == 2)
        {
            distanceLabel.Text = "遥控：原地左转（松开即停车）";
            SendDual(rightRpm, -leftRpm);
        }
        else if (direction == 3)
        {
            distanceLabel.Text = "遥控：原地右转（松开即停车）";
            SendDual(-rightRpm, leftRpm);
        }
        else if (direction == 4)
        {
            distanceLabel.Text = "遥控：前进中向左走弧线";
            SendDual(rightRpm, leftRpm * CurveInnerWheelScale);
        }
        else if (direction == 5)
        {
            distanceLabel.Text = "遥控：前进中向右走弧线";
            SendDual(rightRpm * CurveInnerWheelScale, leftRpm);
        }
        else if (direction == -2)
        {
            distanceLabel.Text = "遥控：后退中向左走弧线";
            SendDual(-rightRpm * CurveInnerWheelScale, -leftRpm);
        }
        else if (direction == -3)
        {
            distanceLabel.Text = "遥控：后退中向右走弧线";
            SendDual(-rightRpm, -leftRpm * CurveInnerWheelScale);
        }
    }

    private void EndManualDirection()
    {
        if (heldDirection == 0) return;
        heldDirection = 0;
        Send("STOP", true);
        distanceLabel.Text = "遥控：已松开，停车";
    }

    private void StartDistance(int direction)
    {
        double rightRpm, leftRpm, distance;
        if (!ReadWheelRpm(out rightRpm, out leftRpm)) return;
        if (!TryNumber(distanceBox.Text, "距离（米）", 0.02, MaxDistanceM, out distance)) return;
        if (!SendDual(direction * rightRpm, direction * leftRpm)) return;
        distanceDirection = direction;
        distanceTarget = distance;
        distanceDone = 0;
        distanceRunning = true;
        lastDistanceSample = null;
        progress.Value = 0;
        distanceLabel.Text = "里程任务：" + (direction > 0 ? "前进 " : "后退 ") + distance.ToString("0.00") + " m，正在执行";
    }

    private bool SendDual(double rightRpm, double leftRpm)
    {
        return Send("DUAL," + rightRpm.ToString("0.###", CultureInfo.InvariantCulture) + "," + leftRpm.ToString("0.###", CultureInfo.InvariantCulture), false);
    }

    private void EmergencyStop()
    {
        distanceRunning = false;
        ClearDriveKeys();
        heldDirection = 0;
        Send("STOP", true);
        progress.Value = 0;
        distanceLabel.Text = "里程任务：已急停";
    }

    private void SerialDataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        try
        {
            int count = serial.BytesToRead;
            if (count <= 0) return;
            byte[] chunk = new byte[count];
            serial.Read(chunk, 0, count);
            lock (receiveLock)
            {
                receiveBuffer.AddRange(chunk);
                ParsePackets();
            }
        }
        catch { }
    }

    private void ParsePackets()
    {
        byte[] magic = Encoding.ASCII.GetBytes("MTR2");
        while (true)
        {
            int marker = FindMagic(receiveBuffer, magic);
            if (marker < 0)
            {
                if (receiveBuffer.Count > 3) receiveBuffer.RemoveRange(0, receiveBuffer.Count - 3);
                return;
            }
            if (marker > 0) receiveBuffer.RemoveRange(0, marker);
            if (receiveBuffer.Count < 90) return;
            byte[] packet = receiveBuffer.GetRange(0, 90).ToArray();
            ushort receivedCrc = BitConverter.ToUInt16(packet, 88);
            if (receivedCrc != Crc16(packet, 4, 84))
            {
                receiveBuffer.RemoveAt(0);
                continue;
            }
            MotorSample sample = new MotorSample();
            sample.Time = DateTime.UtcNow;
            sample.Sequence = BitConverter.ToUInt16(packet, 4);
            sample.Mode = packet[6];
            sample.RightTarget = BitConverter.ToSingle(packet, 8);
            sample.RightActual = BitConverter.ToSingle(packet, 12);
            sample.RightPwm = BitConverter.ToSingle(packet, 20);
            sample.LeftTarget = BitConverter.ToSingle(packet, 24);
            sample.LeftActual = BitConverter.ToSingle(packet, 28);
            sample.LeftPwm = BitConverter.ToSingle(packet, 36);
            samples.Enqueue(sample);
            receiveBuffer.RemoveRange(0, 90);
        }
    }

    private static int FindMagic(List<byte> data, byte[] magic)
    {
        for (int i = 0; i <= data.Count - magic.Length; i++)
        {
            bool match = true;
            for (int j = 0; j < magic.Length; j++) if (data[i + j] != magic[j]) { match = false; break; }
            if (match) return i;
        }
        return -1;
    }

    private static ushort Crc16(byte[] data, int offset, int count)
    {
        ushort crc = 0xFFFF;
        for (int i = offset; i < offset + count; i++)
        {
            crc ^= (ushort)(data[i] << 8);
            for (int bit = 0; bit < 8; bit++) crc = (ushort)(((crc & 0x8000) != 0) ? (crc << 1) ^ 0x1021 : crc << 1);
        }
        return crc;
    }

    private void TimerTick(object sender, EventArgs e)
    {
        if (IsConnected && (DateTime.Now - lastKeepAlive).TotalMilliseconds >= 250)
        {
            Send("KEEP", true);
            lastKeepAlive = DateTime.Now;
        }
        MotorSample sample;
        MotorSample latest = null;
        while (samples.TryDequeue(out sample))
        {
            latest = sample;
            UpdateDistance(sample);
            plot.Add(sample.RightActual, sample.LeftActual);
        }
        if (latest != null) UpdateLabels(latest);
    }

    private void UpdateLabels(MotorSample sample)
    {
        string[] names = { "停止", "右轮闭环", "左轮闭环", "CCD 循迹", "双轮闭环", "右轮开环", "左轮开环" };
        modeLabel.Text = "模式：" + (sample.Mode < names.Length ? names[sample.Mode] : sample.Mode.ToString());
        rightLabel.Text = string.Format(CultureInfo.InvariantCulture, "右轮 {0,7:+0.0;-0.0;0.0} / {1,7:+0.0;-0.0;0.0} RPM   PWM {2,6:+0.0;-0.0;0.0}%", sample.RightActual, sample.RightTarget, sample.RightPwm);
        leftLabel.Text = string.Format(CultureInfo.InvariantCulture, "左轮 {0,7:+0.0;-0.0;0.0} / {1,7:+0.0;-0.0;0.0} RPM   PWM {2,6:+0.0;-0.0;0.0}%", sample.LeftActual, sample.LeftTarget, sample.LeftPwm);
        if (IsConnected) connectionLabel.Text = "已连接 " + serial.PortName + "；遥测 " + sample.Sequence;
    }

    private void UpdateDistance(MotorSample sample)
    {
        if (!distanceRunning)
        {
            lastDistanceSample = sample.Time;
            return;
        }
        if (!lastDistanceSample.HasValue)
        {
            lastDistanceSample = sample.Time;
            return;
        }
        double dt = Math.Min(0.25, (sample.Time - lastDistanceSample.Value).TotalSeconds);
        lastDistanceSample = sample.Time;
        double averageRpm = (Math.Abs(sample.RightActual) + Math.Abs(sample.LeftActual)) * 0.5;
        distanceDone += averageRpm / 60.0 * Math.PI * WheelDiameterM * dt;
        int percent = (int)Math.Min(100, Math.Round(100.0 * distanceDone / distanceTarget));
        progress.Value = Math.Max(0, Math.Min(100, percent));
        double remaining = Math.Max(0, distanceTarget - distanceDone);
        distanceLabel.Text = string.Format(CultureInfo.InvariantCulture, "里程任务：{0} {1:0.000} / {2:0.000} m，剩余 {3:0.000} m", distanceDirection > 0 ? "前进" : "后退", distanceDone, distanceTarget, remaining);
        if (distanceDone >= distanceTarget)
        {
            distanceRunning = false;
            Send("STOP", true);
            progress.Value = 100;
            distanceLabel.Text = "里程任务：完成，估算行驶 " + distanceDone.ToString("0.000", CultureInfo.InvariantCulture) + " m（已自动停车）";
        }
    }
}

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new CarRemoteForm());
    }
}
