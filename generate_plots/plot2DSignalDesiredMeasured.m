function  plot2DSignalDesiredMeasured(time, desired, measured, domain, yLimit,yAxisLabel)
plot(time, desired);
hold on
plot(time, measured);

xlim(domain)
ylim(yLimit)
plot_aesthetic('', 'Time (s)',  yAxisLabel, '', 'x desired', 'y desired', 'x measured', 'y measured')

end

