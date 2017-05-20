load('bounce_02_09z.mat', 't', 'z')
z = z - 18;

%% Raw bounce
figure
ax = gca;
hp = plot(t, z, '.-');
ylabel('Height above ground, mm')
xlabel('Time, s')
hp.MarkerSize = 10;
hp.LineWidth = 0.7;
ax.FontSize = 14;
xlim([0, max(t)])

%% Demonstration of transform
R = 0.68;
z_sub = z(31:70);
t_sub = t(31:70);
z_before = z_sub(1:22);
t_before = t_sub(1:22);
z_after = z_sub(23:end);
t_after = t_sub(23:end);

f1 = fit(t_before, z_before, 'poly2');
r1 = roots([f1.p1, f1.p2, f1.p3]);
t_bounce = max(r1);
t_bounce = 1.052;

t_rescaled = t_bounce - (t_after - t_bounce) ./ sqrt(R);
z_rescaled = z_after ./ R;

figure
hp1 = plot(t_sub, z_sub, '.-');
hold on
hp2 = plot(t_rescaled, z_rescaled, 'x');
plot(t_bounce, 0, 'ko');
hold off

ax = gca;
ylabel('Height above ground, mm')
xlabel('Time, s')
legend('Original data', 'Transformed second bounce', 'Predicted bounce location')
hp1.MarkerSize = 10;
hp1.LineWidth = 0.7;
ax.FontSize = 14;
xlim([min(t_sub), max(t_sub)])
