$(document).ready(function(){
	setInterval(function(){
		$.getJSON("http://localhost:9000/status", function(result){
		    // $("#statusmsg").html(result["status"]);
			// $("#csvfile").html(result["csvfile"]);
			// $("#zipfile").html(result["zipfile"]);
			// $("#cam1_count").html(result["cam1"]["image_count"]);
			// $("#cam1_img").html(result["cam1"]["current_image"]);
			// $("#cam1_pending").html(result["cam1"]["pending_dl"]);
			// $("#cam1_lastact").html(parseInt(result["cam1"]["last_activity"]) + " seconds ago");
			// $("#cam2_count").html(result["cam2"]["image_count"]);
			// $("#cam2_img").html(result["cam2"]["current_image"]);
			// $("#cam2_pending").html(result["cam2"]["pending_dl"]);
			// $("#cam2_lastact").html(parseInt(result["cam2"]["last_activity"]) + " seconds ago");
		})
			.fail( function() {
				$("#statusmsg").html("Not running")
				// $("#csvfile,#zipfile,#cam1_count,#cam1_img,#cam1_pending,#cam1_lastact,#cam2_count,#cam2_img,#cam2_pending,#cam2_lastact").html("");

			});
	}, 2000);
});
