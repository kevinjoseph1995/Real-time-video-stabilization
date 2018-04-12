clc;
clear;
close all;
I_t=double(rgb2gray(imread('3.jpg')));
I_t_minus_1=double(rgb2gray(imread('4.jpg')));
M=50;
N=50;
MAD=zeros(M,N);
for p=1:M
    for q=1:N
        sum=0;
        for k=1:M
            for l=1:N
                sum=sum+abs((I_t_minus_1(500+k,500+l))-(I_t(500+k+p,500+l+q)));
            end 
        end
        MAD(p,q)=sum/(M*N);
    end
end
[minVal, idx] = min(MAD(:));
[u(1), v(1)] = ind2sub(size(MAD), idx);
MAD=zeros(M,N);
for p=1:M
    for q=1:N
        sum=0;
        for k=1:M
            for l=1:N
                sum=sum+abs((I_t_minus_1(150+k,150+l))-(I_t(150+k+p,150+l+q)));
            end 
        end
        MAD(p,q)=sum/(M*N);
    end
end
[minVal, idx] = min(MAD(:));
[u(2), v(2)] = ind2sub(size(MAD), idx);
MAD=zeros(M,N);
for p=1:M
    for q=1:N
        sum=0;
        for k=1:M
            for l=1:N
                sum=sum+abs((I_t_minus_1(0+k,0+l))-(I_t(0+k+p,0+l+q)));
            end 
        end
        MAD(p,q)=sum/(M*N);
    end
end
[minVal, idx] = min(MAD(:));
[u(3), v(3)] = ind2sub(size(MAD), idx);
quiver([500 150 0],[500 150 0],u,v);
