function output = sample_normal(input)
sum = 0;
for i = 1:12
    sum = sum + (2*rand()-1);
end
output = (input/6)*sum;
end
