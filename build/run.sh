for i in ../../dataset_ori/*.obj
	do (./X_ray_trial $i > output_$(basename $i obj)txt)
done
