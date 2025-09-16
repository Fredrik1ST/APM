
namespace APM_Running_App
{
    //Some of this code is inspired from Roberto Filho's Youtube video: https://www.youtube.com/watch?v=yj_dr9HvnwA&t=137s made in  28.12.2021.
    //Accessed 26.03.2024

    public partial class RunStartedPage : ContentPage
    {
        private TimeOnly time = new();
        private bool isRunning;
        List<Entry> entry = new List<Entry>();
        private string text1;
        private string text2;
        private string text3;
        private bool firstime =true;

        public RunStartedPage(string text1, string text2, string text3)
        {
            this.text1 = text1;
            this.text2 = text2;
            this.text3 = text3;

            InitializeComponent();
            Timer();
            entry.Clear();

            if (firstime)
            {
                ChangeEntry(EntriesStackLayout, text1, text2, text3);
                firstime = false;
            }

        }

        private void SetTime()
        {
            timeLabel.Text = $"{time.Minute:00}:{time.Second:00}";
        }

        private async void Timer()
        {
            while (!isRunning)
            {
                time = time.Add(TimeSpan.FromSeconds(1));
                SetTime();
                await Task.Delay(TimeSpan.FromSeconds(1));
            }
        }

        async void Button_Clicked(object sender, EventArgs e)
        {
            isRunning = true;
            var leave = await DisplayAlert("Alert", "You stopped the run. Do you want to save the run?", "Yes", "No");
            if (leave)
            {
                // Save result
                await Navigation.PopToRootAsync();
            }
            else
            {
                await Navigation.PopToRootAsync();
                //await Navigation.PushModalAsync(new PreviousResultsPage());
            }
        }

        private void ChangeEntry(StackLayout sl, string name1, string name2, string name3)
        {
            foreach (var entry in entry)
            {
                entry.Text = "";
            }

            EntryTag1.Text = name1;
            EntryTag_time1.Text = name2;
            EntryTag_time2.Text = name3;
        }

        //protected override bool OnBackButtonPressed()
        //{
        //    Device.BeginInvokeOnMainThread(async () =>
        //    {
        //        var exit = await this.DisplayAlert("Confirm Exit", "Do you really want to exit the application?", "Yes", "No").ConfigureAwait(false);

        //        if (exit)
        //            System.Diagnostics.Process.GetCurrentProcess().CloseMainWindow();
        //    });
        //    return true;
        //}




    }

}
