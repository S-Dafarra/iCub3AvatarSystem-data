function  plot3DSignalDesiredMeasured(time, desired, measured, domain, yLimit,yAxisLabel)
plot(time, desired);
hold on
plot(time, measured);

xlim(domain)
ylim(yLimit)
plot_aesthetic('', 'Time (s)',  yAxisLabel, '', 'x Desired', 'y Desired', 'z Desired', 'x Measured', 'y Measured', 'z Measured')

end

