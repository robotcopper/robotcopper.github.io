$("#toggle2").click(function () {
  $(".sidebar-wrap-2,.sidebar-wrap,.content-wrap, .archive").toggleClass("shift2");
});

$("#toggle").click(function () {
  $(".sidebar-wrap-2").toggleClass("shiftbis");
});

document.addEventListener('click', function(event) {
  if (event.target.classList.contains('lightbox')) {
      window.location.hash = '#gallery';
  }
});