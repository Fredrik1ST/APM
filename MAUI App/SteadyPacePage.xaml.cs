namespace APM_Running_App

//Code inspired from https://learn.microsoft.com/en-us/dotnet/maui/user-interface/controls/entry?view=net-maui-8.0
{
    public partial class SteadyPacePage : ContentPage
    {


        public SteadyPacePage()
        {
            InitializeComponent();


            Entry entry1 = new Entry { Placeholder = "Enter Distance (m)" };
            entry1.TextChanged += OnEntryTextChanged;
            entry1.Completed += OnEntryCompleted;

            Entry entry2 = new Entry { Placeholder = "Enter Finished Time (min)" };
            entry2.TextChanged += OnEntryTextChanged;
            entry2.Completed += OnEntryCompleted;
        }
        async void Button_Clicked(object sender, EventArgs e)
        {
            if (entry_time1.Text == "" || entry_time2.Text == "" || entry1.Text == "")
            {
                await DisplayAlert("Alert", "Please fill in all entries", "OK");
            }
            else
            {
                var response = await DisplayAlert("Start Run", "You have selected to start a run, do you want to continue?", "OK", "Cancel");
                if (response == true)
                {
                    await Navigation.PushAsync(new RunStartedPage(entry1.Text, entry_time1.Text, entry_time2.Text));
                }
            }
        }

        void OnEntryTextChanged(object sender, TextChangedEventArgs e)
        {
            string oldText = e.OldTextValue;
            string newText = e.NewTextValue;
            string distance = entry1.Text;
            string time_min = entry_time1.Text;
            string time_sec = entry_time2.Text;
        }

        void OnEntryCompleted(object sender, EventArgs e)
        {
            string text = ((Entry)sender).Text;
        }


    }

}
